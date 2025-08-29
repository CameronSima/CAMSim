package main

import (
	"math"
	"os"
	"testing"
)

// TestForcesMomentsCalculator tests the forces and moments calculation system
func TestForcesMomentsCalculator(t *testing.T) {
	
	// Load P-51D configuration for testing
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		t.Fatalf("Failed to open P-51D XML: %v", err)
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		t.Fatalf("Failed to parse P-51D config: %v", err)
	}
	
	t.Run("Calculator Creation", func(t *testing.T) {
		calc := NewForcesMomentsCalculator(config)
		
		// Check that reference data was extracted
		if calc.Reference.WingArea <= 0 {
			t.Error("Wing area should be positive")
		}
		if calc.Reference.WingSpan <= 0 {
			t.Error("Wing span should be positive")
		}
		if calc.Mass <= 0 {
			t.Error("Mass should be positive")
		}
		
		t.Logf("Reference Data:")
		t.Logf("  Wing Area: %.2f m²", calc.Reference.WingArea)
		t.Logf("  Wing Span: %.2f m", calc.Reference.WingSpan)
		t.Logf("  Chord: %.2f m", calc.Reference.Chord)
		t.Logf("  Mass: %.0f kg", calc.Mass)
		
		// Check inertia tensor
		if calc.Inertia.XX <= 0 || calc.Inertia.YY <= 0 || calc.Inertia.ZZ <= 0 {
			t.Error("Inertia components should be positive")
		}
		
		t.Logf("Inertia Tensor:")
		t.Logf("  Ixx: %.0f kg·m²", calc.Inertia.XX)
		t.Logf("  Iyy: %.0f kg·m²", calc.Inertia.YY)
		t.Logf("  Izz: %.0f kg·m²", calc.Inertia.ZZ)
	})
	
	t.Run("Forces Calculation", func(t *testing.T) {
		calc := NewForcesMomentsCalculator(config)
		
		// Create test state - cruise flight
		state := NewAircraftState()
		state.Altitude = 3000.0    // 3km altitude
		state.Velocity = Vector3{X: 120.0, Y: 0, Z: 0} // 120 m/s forward
		state.Alpha = 5.0 * DEG_TO_RAD  // 5° angle of attack
		state.Controls.Throttle = 0.75   // 75% power
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		components, err := calc.CalculateForcesMoments(state)
		if err != nil {
			t.Fatalf("Forces calculation failed: %v", err)
		}
		
		// Check that forces are reasonable
		t.Logf("Aerodynamic Forces:")
		t.Logf("  Lift: %.1f N", components.Aerodynamic.Lift)
		t.Logf("  Drag: %.1f N", components.Aerodynamic.Drag)
		t.Logf("  Side: %.1f N", components.Aerodynamic.Side)
		
		// Lift should be negative (upward in body frame)
		if components.Aerodynamic.Lift >= 0 {
			t.Error("Lift should be negative (upward) in body frame")
		}
		
		// Drag should be negative (opposing motion)
		if components.Aerodynamic.Drag >= 0 {
			t.Error("Drag should be negative (opposing motion)")
		}
		
		// Side force should be small for symmetric flight
		if math.Abs(components.Aerodynamic.Side) > math.Abs(components.Aerodynamic.Lift)*0.1 {
			t.Error("Side force should be small for symmetric flight")
		}
		
		t.Logf("Propulsive Forces:")
		t.Logf("  Thrust: %.1f N", components.Propulsion.Thrust)
		t.Logf("  Torque: %.1f N·m", components.Propulsion.Torque)
		
		// Thrust should be positive (forward)
		if components.Propulsion.Thrust <= 0 {
			t.Error("Thrust should be positive")
		}
		
		// Thrust should be proportional to throttle
		expectedThrust := state.Controls.Throttle * 8000.0 * (state.Density / 1.225)
		assertApproxEqual(t, components.Propulsion.Thrust, expectedThrust, expectedThrust*0.1)
		
		t.Logf("Gravitational Force:")
		t.Logf("  Weight: (%.1f, %.1f, %.1f) N", 
			components.Gravity.Weight.X, 
			components.Gravity.Weight.Y, 
			components.Gravity.Weight.Z)
		
		// Weight magnitude should equal mg
		expectedWeight := calc.Mass * 9.81
		actualWeight := components.Gravity.Weight.Magnitude()
		assertApproxEqual(t, actualWeight, expectedWeight, 0.1)
	})
	
	t.Run("Moments Calculation", func(t *testing.T) {
		calc := NewForcesMomentsCalculator(config)
		
		state := NewAircraftState()
		state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
		state.Alpha = 5.0 * DEG_TO_RAD
		state.Controls.Aileron = 0.2   // 20% aileron input
		state.Controls.Elevator = -0.1 // 10% down elevator
		state.Controls.Rudder = 0.05   // 5% rudder
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		components, err := calc.CalculateForcesMoments(state)
		if err != nil {
			t.Fatalf("Moments calculation failed: %v", err)
		}
		
		t.Logf("Moments:")
		t.Logf("  Roll (L): %.1f N·m", components.Moments.Roll)
		t.Logf("  Pitch (M): %.1f N·m", components.Moments.Pitch)
		t.Logf("  Yaw (N): %.1f N·m", components.Moments.Yaw)
		
		// Moments should be non-zero with control inputs
		if math.Abs(components.Moments.Roll) == 0 && state.Controls.Aileron != 0 {
			t.Error("Roll moment should be non-zero with aileron input")
		}
		
		if math.Abs(components.Moments.Pitch) == 0 && state.Controls.Elevator != 0 {
			t.Error("Pitch moment should be non-zero with elevator input")
		}
		
		// Total forces and moments should be computed
		t.Logf("Total Force: (%.1f, %.1f, %.1f) N", 
			components.TotalForce.X, 
			components.TotalForce.Y, 
			components.TotalForce.Z)
		t.Logf("Total Moment: (%.1f, %.1f, %.1f) N·m", 
			components.TotalMoment.X, 
			components.TotalMoment.Y, 
			components.TotalMoment.Z)
	})
	
	t.Run("State Derivatives", func(t *testing.T) {
		calc := NewForcesMomentsCalculator(config)
		
		state := NewAircraftState()
		state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
		state.Controls.Throttle = 1.0 // Full power
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		components, err := calc.CalculateForcesMoments(state)
		if err != nil {
			t.Fatalf("Forces calculation failed: %v", err)
		}
		
		derivatives := calc.CalculateStateDerivatives(state, components)
		
		t.Logf("State Derivatives:")
		t.Logf("  Acceleration: (%.2f, %.2f, %.2f) m/s²", 
			derivatives.VelocityDot.X, 
			derivatives.VelocityDot.Y, 
			derivatives.VelocityDot.Z)
		t.Logf("  Angular Accel: (%.4f, %.4f, %.4f) rad/s²", 
			derivatives.AngularRateDot.X, 
			derivatives.AngularRateDot.Y, 
			derivatives.AngularRateDot.Z)
		t.Logf("  Altitude Rate: %.2f m/s", derivatives.AltitudeDot)
		t.Logf("  Fuel Flow: %.6f kg/s", -derivatives.MassDot)
		
		// Check that derivatives are reasonable
		expectedAccel := components.TotalForce.Scale(1.0 / calc.Mass)
		assertApproxEqual(t, derivatives.VelocityDot.X, expectedAccel.X, 0.01)
		assertApproxEqual(t, derivatives.VelocityDot.Y, expectedAccel.Y, 0.01)
		assertApproxEqual(t, derivatives.VelocityDot.Z, expectedAccel.Z, 0.01)
	})
}

// TestFlightDynamicsEngine tests the complete flight dynamics system
func TestFlightDynamicsEngine(t *testing.T) {
	
	// Load P-51D configuration
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		t.Fatalf("Failed to open P-51D XML: %v", err)
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		t.Fatalf("Failed to parse P-51D config: %v", err)
	}
	
	t.Run("Engine Creation", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		engine := NewFlightDynamicsEngine(config, integrator)
		
		if engine.Calculator == nil {
			t.Error("Calculator should not be nil")
		}
		if engine.Integrator == nil {
			t.Error("Integrator should not be nil")
		}
		if engine.Statistics == nil {
			t.Error("Statistics should not be nil")
		}
	})
	
	t.Run("Single Step Simulation", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		engine := NewFlightDynamicsEngine(config, integrator)
		
		// Initial state
		state := NewAircraftState()
		state.Velocity = Vector3{X: 80.0, Y: 0, Z: 0}
		state.Controls.Throttle = 0.8
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		initialSpeed := state.TrueAirspeed
		initialAlt := state.Altitude
		
		// Take one simulation step
		dt := 0.01
		newState, err := engine.Step(state, dt)
		if err != nil {
			t.Fatalf("Simulation step failed: %v", err)
		}
		
		// Check that state evolved
		if newState.Time <= state.Time {
			t.Error("Time should advance")
		}
		
		if newState.Position.X <= state.Position.X {
			t.Error("Aircraft should move forward")
		}
		
		t.Logf("State Evolution:")
		t.Logf("  Speed: %.1f → %.1f m/s", initialSpeed, newState.TrueAirspeed)
		t.Logf("  Altitude: %.1f → %.1f m", initialAlt, newState.Altitude)
		t.Logf("  Position: (%.3f, %.3f, %.3f) m", 
			newState.Position.X, newState.Position.Y, newState.Position.Z)
		
		// Check that forces are stored in state
		if newState.Forces.Total.Magnitude() == 0 {
			t.Error("Total force should be stored in state")
		}
		if newState.Moments.Total.Magnitude() == 0 {
			t.Error("Total moment should be stored in state")
		}
	})
	
	t.Run("Multi-Step Flight Simulation", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		engine := NewFlightDynamicsEngine(config, integrator)
		
		// Initial cruise conditions
		state := NewAircraftState()
		state.Altitude = 3000.0
		state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
		state.Controls.Throttle = 0.7
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		initialAlt := state.Altitude
		
		// Simulate 5 seconds of flight
		dt := 0.01
		steps := 500
		
		for i := 0; i < steps; i++ {
			newState, err := engine.Step(state, dt)
			if err != nil {
				t.Fatalf("Step %d failed: %v", i, err)
			}
			state = newState
		}
		
		t.Logf("5-Second Flight Results:")
		t.Logf("  Final Speed: %.1f m/s (%.1f kt)", state.TrueAirspeed, state.TrueAirspeed*MS_TO_KT)
		t.Logf("  Final Altitude: %.1f m (%.0f ft)", state.Altitude, state.Altitude*M_TO_FT)
		t.Logf("  Distance Traveled: %.1f m", state.Position.X)
		t.Logf("  Altitude Change: %.1f m", state.Altitude-initialAlt)
		
		// Aircraft should have traveled a reasonable distance
		expectedDistance := 100.0 * 5.0 // Speed * time (rough estimate)
		if state.Position.X < expectedDistance*0.8 || state.Position.X > expectedDistance*1.2 {
			t.Errorf("Distance traveled %.1f m not in expected range around %.1f m", 
				state.Position.X, expectedDistance)
		}
		
		// Check performance statistics
		report := engine.GetPerformanceReport()
		t.Logf("Performance Report:\n%s", report)
		
		if engine.Statistics.FlightTime <= 0 {
			t.Error("Flight time should be positive")
		}
		if engine.Statistics.MaxSpeed <= 0 {
			t.Error("Max speed should be positive")
		}
	})
	
	t.Run("Climbing Flight", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		engine := NewFlightDynamicsEngine(config, integrator)
		
		// Set up for climb
		state := NewAircraftState()
		state.Velocity = Vector3{X: 80.0, Y: 0, Z: -5.0} // 5 m/s climb
		state.Controls.Throttle = 1.0 // Full power
		state.Controls.Elevator = 0.1 // Slight up elevator
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		initialAlt := state.Altitude
		
		// Simulate climb for 10 seconds
		dt := 0.01
		steps := 1000
		
		for i := 0; i < steps; i++ {
			newState, err := engine.Step(state, dt)
			if err != nil {
				t.Fatalf("Climb step %d failed: %v", i, err)
			}
			state = newState
		}
		
		altGain := state.Altitude - initialAlt
		avgClimbRate := altGain / 10.0
		
		t.Logf("Climb Performance:")
		t.Logf("  Altitude Gain: %.1f m (%.0f ft)", altGain, altGain*M_TO_FT)
		t.Logf("  Average Climb Rate: %.2f m/s (%.0f ft/min)", avgClimbRate, avgClimbRate*60*M_TO_FT)
		t.Logf("  Final Speed: %.1f m/s", state.TrueAirspeed)
		
		// Should have climbed significantly
		if altGain <= 0 {
			t.Error("Aircraft should have climbed")
		}
		
		// Climb rate should be reasonable for a fighter aircraft
		if avgClimbRate < 1.0 || avgClimbRate > 50.0 {
			t.Errorf("Climb rate %.2f m/s seems unrealistic", avgClimbRate)
		}
	})
	
	t.Run("Banking Turn", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		engine := NewFlightDynamicsEngine(config, integrator)
		
		// Set up for turn
		state := NewAircraftState()
		state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
		state.Controls.Throttle = 0.8
		state.Controls.Aileron = 0.3  // Banking input
		state.Controls.Rudder = 0.1   // Coordinated turn
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		initialHeading := state.Yaw
		
		// Simulate turn for 3 seconds
		dt := 0.01
		steps := 300
		
		for i := 0; i < steps; i++ {
			newState, err := engine.Step(state, dt)
			if err != nil {
				t.Fatalf("Turn step %d failed: %v", i, err)
			}
			state = newState
		}
		
		headingChange := math.Abs(state.Yaw - initialHeading)
		
		t.Logf("Turn Performance:")
		t.Logf("  Heading Change: %.1f° in 3 seconds", headingChange*RAD_TO_DEG)
		t.Logf("  Turn Rate: %.1f°/s", (headingChange*RAD_TO_DEG)/3.0)
		t.Logf("  Final Roll Angle: %.1f°", state.Roll*RAD_TO_DEG)
		
		// Should have turned significantly
		if headingChange < 5.0*DEG_TO_RAD {
			t.Error("Aircraft should have turned more with control inputs")
		}
		
		// Should have developed some bank angle
		if math.Abs(state.Roll) < 1.0*DEG_TO_RAD {
			t.Error("Aircraft should have developed bank angle")
		}
	})
}

// TestAerodynamicAnalysis tests the aerodynamic analysis tools
func TestAerodynamicAnalysis(t *testing.T) {
	
	// Load P-51D configuration
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		t.Fatalf("Failed to open P-51D XML: %v", err)
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		t.Fatalf("Failed to parse P-51D config: %v", err)
	}
	
	t.Run("Aerodynamic Analysis", func(t *testing.T) {
		calc := NewForcesMomentsCalculator(config)
		
		// Base state for analysis
		baseState := NewAircraftState()
		baseState.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
		baseState.Altitude = 3000.0
		baseState.UpdateAtmosphere()
		baseState.UpdateDerivedParameters()
		
		analysis := calc.PerformAerodynamicAnalysis(baseState)
		
		t.Logf("Aerodynamic Analysis Results:")
		t.Logf("  Alpha range: %.1f° to %.1f°", 
			analysis.AlphaRange[0]*RAD_TO_DEG, 
			analysis.AlphaRange[len(analysis.AlphaRange)-1]*RAD_TO_DEG)
		t.Logf("  Max L/D ratio: %.2f at α = %.1f°", 
			analysis.MaxLD, analysis.BestAlpha*RAD_TO_DEG)
		t.Logf("  Stall angle: %.1f°", analysis.StallAlpha*RAD_TO_DEG)
		
		// Check that analysis produced reasonable results
		if len(analysis.AlphaRange) != 31 {
			t.Errorf("Expected 31 alpha points, got %d", len(analysis.AlphaRange))
		}
		
		if analysis.MaxLD <= 0 {
			t.Error("Max L/D should be positive")
		}
		
		// L/D should be reasonable for fighter aircraft (10-20 typical)
		if analysis.MaxLD < 5.0 || analysis.MaxLD > 30.0 {
			t.Errorf("Max L/D %.2f seems unrealistic", analysis.MaxLD)
		}
		
		// Best alpha should be positive but not too high
		bestAlphaDeg := analysis.BestAlpha * RAD_TO_DEG
		if bestAlphaDeg < 0 || bestAlphaDeg > 15.0 {
			t.Errorf("Best alpha %.1f° seems unrealistic", bestAlphaDeg)
		}
		
		// Print some curve points for inspection
		t.Logf("CL-Alpha Curve (sample points):")
		for i := 0; i < len(analysis.AlphaRange); i += 5 {
			t.Logf("  α=%.1f°: CL=%.3f, CD=%.4f, L/D=%.1f", 
				analysis.AlphaRange[i]*RAD_TO_DEG,
				analysis.CLCurve[i],
				analysis.CDCurve[i],
				analysis.LDRatio[i])
		}
	})
	
	t.Run("Performance Envelope", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		engine := NewFlightDynamicsEngine(config, integrator)
		
		envelope := engine.CalculatePerformanceEnvelope()
		
		t.Logf("Performance Envelope:")
		t.Logf("  Service Ceiling: %.0f m (%.0f ft)", 
			envelope.ServiceCeiling, envelope.ServiceCeiling*M_TO_FT)
		t.Logf("  Absolute Ceiling: %.0f m (%.0f ft)", 
			envelope.AbsoluteCeiling, envelope.AbsoluteCeiling*M_TO_FT)
		
		t.Logf("Performance by Altitude:")
		for i, alt := range envelope.Altitudes {
			t.Logf("  %.0fm: Max Speed %.1f m/s (%.1f kt), Climb %.1f m/s", 
				alt, 
				envelope.MaxSpeeds[i], envelope.MaxSpeeds[i]*MS_TO_KT,
				envelope.ClimbRates[i])
		}
		
		// Check reasonable performance values
		if envelope.ServiceCeiling <= 0 {
			t.Error("Service ceiling should be positive")
		}
		
		// P-51D service ceiling should be around 12,000m
		if envelope.ServiceCeiling < 8000 || envelope.ServiceCeiling > 15000 {
			t.Errorf("Service ceiling %.0fm seems unrealistic for P-51D", envelope.ServiceCeiling)
		}
		
		// Performance should decrease with altitude
		for i := 1; i < len(envelope.ClimbRates); i++ {
			if envelope.ClimbRates[i] > envelope.ClimbRates[i-1] {
				t.Error("Climb rate should generally decrease with altitude")
			}
		}
	})
}

// BenchmarkForcesMomentsCalculation benchmarks the forces/moments computation
func BenchmarkForcesMomentsCalculation(b *testing.B) {
	
	// Load P-51D configuration
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		b.Fatalf("Failed to open P-51D XML: %v", err)
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		b.Fatalf("Failed to parse P-51D config: %v", err)
	}
	
	calc := NewForcesMomentsCalculator(config)
	
	// Test state
	state := NewAircraftState()
	state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
	state.Controls.Throttle = 0.8
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	b.ResetTimer()
	
	for i := 0; i < b.N; i++ {
		_, err := calc.CalculateForcesMoments(state)
		if err != nil {
			b.Fatalf("Forces calculation failed: %v", err)
		}
	}
}

// BenchmarkFlightDynamicsStep benchmarks a complete simulation step
func BenchmarkFlightDynamicsStep(b *testing.B) {
	
	// Load P-51D configuration
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		b.Fatalf("Failed to open P-51D XML: %v", err)
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		b.Fatalf("Failed to parse P-51D config: %v", err)
	}
	
	integrator := NewRungeKutta4Integrator()
	engine := NewFlightDynamicsEngine(config, integrator)
	
	// Test state
	state := NewAircraftState()
	state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
	state.Controls.Throttle = 0.8
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	dt := 0.01
	
	b.ResetTimer()
	
	for i := 0; i < b.N; i++ {
		newState, err := engine.Step(state, dt)
		if err != nil {
			b.Fatalf("Simulation step failed: %v", err)
		}
		state = newState
	}
}
