package main

import (
	"math"
	"testing"
)

// TestEulerIntegrator tests the Euler integration method
func TestEulerIntegrator(t *testing.T) {
	
	t.Run("Basic Integration", func(t *testing.T) {
		integrator := NewEulerIntegrator()
		
		// Test basic properties
		assertEqual(t, integrator.GetName(), "Euler")
		assertEqual(t, integrator.GetOrder(), 1)
		
		// Create test state
		state := NewAircraftState()
		state.Velocity = Vector3{X: 100.0, Y: 0.0, Z: 0.0} // 100 m/s forward
		state.AngularRate = Vector3{X: 0.1, Y: 0.0, Z: 0.0} // 0.1 rad/s roll rate
		
		// Create test derivatives (constant acceleration)
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 1.0, Y: 0.0, Z: 0.0}, // 1 m/s² forward acceleration
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0}, // No angular acceleration
		}
		
		dt := 0.01 // 10ms time step
		
		// Integrate one step
		newState := integrator.Integrate(state, derivatives, dt)
		
		// Check time advancement
		assertEqual(t, newState.Time, state.Time+dt)
		
		// Check velocity integration: v_new = v_old + a*dt
		expectedVel := 100.0 + 1.0*dt
		assertApproxEqual(t, newState.Velocity.X, expectedVel, 0.001)
		assertEqual(t, newState.Velocity.Y, 0.0)
		assertEqual(t, newState.Velocity.Z, 0.0)
		
		// Check position integration (should move forward)
		if newState.Position.X <= state.Position.X {
			t.Error("Aircraft should move forward")
		}
		
		// Check that derived parameters are updated
		if newState.TrueAirspeed <= state.TrueAirspeed {
			t.Error("True airspeed should increase with acceleration")
		}
	})
	
	t.Run("Constant Velocity Flight", func(t *testing.T) {
		integrator := NewEulerIntegrator()
		
		// Level flight with constant velocity
		state := NewAircraftState()
		state.Velocity = Vector3{X: 50.0, Y: 0.0, Z: 0.0}
		initialPos := state.Position
		
		// No accelerations (cruise flight)
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 0.0, Y: 0.0, Z: 0.0},
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		}
		
		dt := 0.01
		steps := 100 // 1 second of flight
		
		for i := 0; i < steps; i++ {
			state = integrator.Integrate(state, derivatives, dt)
		}
		
		// Should travel 50 m in 1 second
		distanceTraveled := state.Position.Add(initialPos.Scale(-1)).Magnitude()
		assertApproxEqual(t, distanceTraveled, 50.0, 0.1)
		
		// Velocity should remain constant
		assertApproxEqual(t, state.Velocity.X, 50.0, 0.001)
	})
	
	t.Run("Rolling Motion", func(t *testing.T) {
		integrator := NewEulerIntegrator()
		
		state := NewAircraftState()
		state.AngularRate = Vector3{X: 0.5, Y: 0.0, Z: 0.0} // 0.5 rad/s roll rate
		
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 0.0, Y: 0.0, Z: 0.0},
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0}, // Constant roll rate
		}
		
		dt := 0.01
		steps := int(math.Pi / (0.5 * dt)) // Time for 180° roll
		
		initialRoll := state.Roll
		
		for i := 0; i < steps; i++ {
			state = integrator.Integrate(state, derivatives, dt)
		}
		
		// Should have rolled approximately 180°
		rollChange := math.Abs(state.Roll - initialRoll)
		assertApproxEqual(t, rollChange, math.Pi, 0.1)
	})
}

// TestRungeKutta4Integrator tests the RK4 integration method
func TestRungeKutta4Integrator(t *testing.T) {
	
	t.Run("Basic Properties", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		
		assertEqual(t, integrator.GetName(), "Runge-Kutta 4th Order")
		assertEqual(t, integrator.GetOrder(), 4)
	})
	
	t.Run("Accuracy Comparison with Euler", func(t *testing.T) {
		// Compare RK4 vs Euler for same scenario
		euler := NewEulerIntegrator()
		rk4 := NewRungeKutta4Integrator()
		
		// Start with same initial conditions
		initialState := NewAircraftState()
		initialState.Velocity = Vector3{X: 50.0, Y: 0.0, Z: 0.0}
		
		// Constant acceleration
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 2.0, Y: 0.0, Z: 0.0},
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		}
		
		dt := 0.1 // Larger time step to amplify differences
		steps := 10
		
		// Integrate with both methods
		eulerState := initialState.Copy()
		rk4State := initialState.Copy()
		
		for i := 0; i < steps; i++ {
			eulerState = euler.Integrate(eulerState, derivatives, dt)
			rk4State = rk4.Integrate(rk4State, derivatives, dt)
		}
		
		// RK4 should be more accurate (though for constant acceleration, 
		// the difference may be small)
		t.Logf("Euler final velocity: %.3f m/s", eulerState.Velocity.X)
		t.Logf("RK4 final velocity: %.3f m/s", rk4State.Velocity.X)
		
		// Both should be close to analytical solution
		expectedVel := 50.0 + 2.0*float64(steps)*dt
		eulerError := math.Abs(eulerState.Velocity.X - expectedVel)
		rk4Error := math.Abs(rk4State.Velocity.X - expectedVel)
		
		t.Logf("Euler error: %.6f, RK4 error: %.6f", eulerError, rk4Error)
		
		// Both should be reasonably accurate for this simple case
		if eulerError > 1.0 {
			t.Error("Euler integration error too large")
		}
		if rk4Error > 1.0 {
			t.Error("RK4 integration error too large")
		}
	})
	
	t.Run("Stability Test", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		
		// Test with oscillatory motion (simplified harmonic oscillator)
		state := NewAircraftState()
		state.Velocity = Vector3{X: 0.0, Y: 10.0, Z: 0.0} // Initial sideways velocity
		
		dt := 0.01
		steps := 1000 // 10 seconds
		
		maxVelocity := 0.0
		
		for i := 0; i < steps; i++ {
			// Simple harmonic motion derivatives (simplified)
			omega := 1.0 // Angular frequency
			derivatives := &StateDerivatives{
				VelocityDot: Vector3{
					X: 0.0,
					Y: -omega * omega * state.Position.Y, // Spring force
					Z: 0.0,
				},
				AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
			}
			
			state = integrator.Integrate(state, derivatives, dt)
			
			vel := state.Velocity.Magnitude()
			if vel > maxVelocity {
				maxVelocity = vel
			}
		}
		
		// For stable integration, velocity shouldn't grow without bound
		if maxVelocity > 50.0 {
			t.Errorf("Integration appears unstable: max velocity %.3f", maxVelocity)
		}
	})
}

// TestAdamsBashforth2Integrator tests the Adams-Bashforth method
func TestAdamsBashforth2Integrator(t *testing.T) {
	
	t.Run("Basic Properties", func(t *testing.T) {
		integrator := NewAdamsBashforth2Integrator()
		
		assertEqual(t, integrator.GetName(), "Adams-Bashforth 2nd Order")
		assertEqual(t, integrator.GetOrder(), 2)
	})
	
	t.Run("Two Step Integration", func(t *testing.T) {
		integrator := NewAdamsBashforth2Integrator()
		
		state := NewAircraftState()
		state.Velocity = Vector3{X: 30.0, Y: 0.0, Z: 0.0}
		
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 1.0, Y: 0.0, Z: 0.0},
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		}
		
		dt := 0.01
		
		// First step (should use Euler)
		state1 := integrator.Integrate(state, derivatives, dt)
		
		// Second step (should use Adams-Bashforth)
		state2 := integrator.Integrate(state1, derivatives, dt)
		
		// Check that integration is working
		if state2.Time <= state1.Time {
			t.Error("Time should advance")
		}
		
		if state2.Velocity.X <= state1.Velocity.X {
			t.Error("Velocity should increase with positive acceleration")
		}
	})
}

// TestIntegratorComparison tests the comparison framework
func TestIntegratorComparison(t *testing.T) {
	
	t.Run("Compare Methods", func(t *testing.T) {
		comparison := NewIntegratorComparison()
		
		// Check that all methods are included
		if len(comparison.Methods) < 3 {
			t.Error("Should have at least 3 integration methods")
		}
		
		// Run comparison
		initialState := NewAircraftState()
		initialState.Velocity = Vector3{X: 25.0, Y: 0.0, Z: 0.0}
		
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 0.5, Y: 0.0, Z: 0.0},
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		}
		
		dt := 0.01
		steps := 100
		
		results := comparison.CompareIntegrators(initialState, derivatives, dt, steps)
		
		// Check that we get results for all methods
		if len(results) != len(comparison.Methods) {
			t.Errorf("Expected %d results, got %d", len(comparison.Methods), len(results))
		}
		
		// Log results for inspection
		for method, finalState := range results {
			t.Logf("%s: Final velocity = %.3f m/s, Position = (%.3f, %.3f, %.3f)",
				method, 
				finalState.Velocity.X,
				finalState.Position.X,
				finalState.Position.Y,
				finalState.Position.Z)
		}
		
		// All methods should produce reasonable results
		for method, finalState := range results {
			if finalState.Velocity.X <= initialState.Velocity.X {
				t.Errorf("%s: Velocity should increase", method)
			}
			if finalState.Position.X <= initialState.Position.X {
				t.Errorf("%s: Aircraft should move forward", method)
			}
		}
	})
}

// TestStabilityAnalysis tests the stability analysis framework
func TestStabilityAnalysis(t *testing.T) {
	
	t.Run("Analyze Stability", func(t *testing.T) {
		analysis := NewStabilityAnalysis()
		
		initialState := NewAircraftState()
		initialState.Velocity = Vector3{X: 50.0, Y: 0.0, Z: 0.0}
		initialState.Altitude = 1000.0
		
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 0.1, Y: 0.0, Z: 0.0},
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		}
		
		duration := 1.0 // 1 second
		
		analysis.AnalyzeStability(initialState, derivatives, duration)
		
		// Check that we got results for all time steps
		if len(analysis.Results) != len(analysis.TimeSteps) {
			t.Errorf("Expected results for %d time steps, got %d", 
				len(analysis.TimeSteps), len(analysis.Results))
		}
		
		// Log results
		for dt, methodResults := range analysis.Results {
			t.Logf("dt = %.3f:", dt)
			for method, energy := range methodResults {
				t.Logf("  %s: Total energy = %.3f", method, energy)
			}
		}
		
		// Sanity check: energy should be positive for all cases
		for dt, methodResults := range analysis.Results {
			for method, energy := range methodResults {
				if energy <= 0 {
					t.Errorf("dt=%.3f, %s: Energy should be positive, got %.3f", 
						dt, method, energy)
				}
			}
		}
	})
}

// TestAdaptiveTimeStep tests adaptive time stepping
func TestAdaptiveTimeStep(t *testing.T) {
	
	t.Run("Basic Adaptive Integration", func(t *testing.T) {
		baseIntegrator := NewRungeKutta4Integrator()
		adaptive := NewAdaptiveTimeStep(baseIntegrator)
		
		// Check default parameters
		if adaptive.MinDt <= 0 {
			t.Error("MinDt should be positive")
		}
		if adaptive.MaxDt <= adaptive.MinDt {
			t.Error("MaxDt should be greater than MinDt")
		}
		if adaptive.Tolerance <= 0 {
			t.Error("Tolerance should be positive")
		}
		
		state := NewAircraftState()
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 1.0, Y: 0.0, Z: 0.0},
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		}
		
		targetDt := 0.01
		
		newState, suggestedDt := adaptive.AdaptiveIntegrate(state, derivatives, targetDt)
		
		// Check that integration occurred
		if newState.Time <= state.Time {
			t.Error("Time should advance")
		}
		
		// Check that suggested time step is reasonable
		if suggestedDt <= 0 {
			t.Error("Suggested dt should be positive")
		}
		if suggestedDt < adaptive.MinDt || suggestedDt > adaptive.MaxDt {
			t.Errorf("Suggested dt %.6f outside bounds [%.6f, %.6f]", 
				suggestedDt, adaptive.MinDt, adaptive.MaxDt)
		}
		
		t.Logf("Target dt: %.6f, Suggested dt: %.6f", targetDt, suggestedDt)
	})
	
	t.Run("Error Estimation", func(t *testing.T) {
		baseIntegrator := NewRungeKutta4Integrator()
		adaptive := NewAdaptiveTimeStep(baseIntegrator)
		
		state := NewAircraftState()
		state.Velocity = Vector3{X: 100.0, Y: 0.0, Z: 0.0}
		
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 10.0, Y: 0.0, Z: 0.0}, // High acceleration
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		}
		
		smallDt := 0.001
		largeDt := 0.1
		
		smallError := adaptive.EstimateError(state, derivatives, smallDt)
		largeError := adaptive.EstimateError(state, derivatives, largeDt)
		
		// Larger time steps should generally have larger errors
		if largeError < smallError {
			t.Logf("Warning: Large dt error (%.6f) < small dt error (%.6f)", 
				largeError, smallError)
		}
		
		t.Logf("Error with dt=%.3f: %.6f", smallDt, smallError)
		t.Logf("Error with dt=%.3f: %.6f", largeDt, largeError)
	})
}

// TestIntegrationStatistics tests the statistics tracking
func TestIntegrationStatistics(t *testing.T) {
	
	t.Run("Statistics Tracking", func(t *testing.T) {
		stats := NewIntegrationStatistics()
		
		// Simulate some integration steps
		testSteps := []struct {
			dt       float64
			accepted bool
			error    float64
		}{
			{0.01, true, 0.001},
			{0.005, true, 0.0005},
			{0.02, false, 0.01},  // Rejected step
			{0.01, true, 0.001},
			{0.015, true, 0.002},
		}
		
		for _, step := range testSteps {
			stats.UpdateStats(step.dt, step.accepted, step.error)
		}
		
		// Check basic counts
		assertEqual(t, stats.TotalSteps, 5)
		assertEqual(t, stats.AcceptedSteps, 4)
		assertEqual(t, stats.RejectedSteps, 1)
		
		// Check efficiency
		expectedEfficiency := 4.0 / 5.0
		assertApproxEqual(t, stats.GetEfficiency(), expectedEfficiency, 0.01)
		
		// Check that min/max are tracked
		if stats.MinStepSize != 0.005 {
			t.Errorf("Expected min step size 0.005, got %.6f", stats.MinStepSize)
		}
		if stats.MaxStepSize != 0.015 {
			t.Errorf("Expected max step size 0.015, got %.6f", stats.MaxStepSize)
		}
		
		// Test string representation
		statsStr := stats.String()
		if len(statsStr) == 0 {
			t.Error("Statistics string should not be empty")
		}
		
		t.Logf("Statistics:\n%s", statsStr)
	})
}

// BenchmarkIntegrators compares integration method performance
func BenchmarkIntegrators(b *testing.B) {
	
	integrators := []Integrator{
		NewEulerIntegrator(),
		NewRungeKutta4Integrator(),
		NewAdamsBashforth2Integrator(),
	}
	
	// Common test setup
	state := NewAircraftState()
	state.Velocity = Vector3{X: 50.0, Y: 0.0, Z: 0.0}
	
	derivatives := &StateDerivatives{
		VelocityDot:    Vector3{X: 1.0, Y: 0.0, Z: 0.0},
		AngularRateDot: Vector3{X: 0.1, Y: 0.0, Z: 0.0},
	}
	
	dt := 0.01
	
	for _, integrator := range integrators {
		b.Run(integrator.GetName(), func(b *testing.B) {
			testState := state.Copy()
			
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				testState = integrator.Integrate(testState, derivatives, dt)
			}
		})
	}
}

// BenchmarkAdaptiveIntegration benchmarks adaptive time stepping
func BenchmarkAdaptiveIntegration(b *testing.B) {
	
	baseIntegrator := NewRungeKutta4Integrator()
	adaptive := NewAdaptiveTimeStep(baseIntegrator)
	
	state := NewAircraftState()
	derivatives := &StateDerivatives{
		VelocityDot:    Vector3{X: 2.0, Y: 0.0, Z: 0.0},
		AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
	}
	
	targetDt := 0.01
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		state, _ = adaptive.AdaptiveIntegrate(state, derivatives, targetDt)
	}
}

// TestIntegrationRealism tests integration with realistic flight scenarios
func TestIntegrationRealism(t *testing.T) {
	
	t.Run("Realistic Cruise Flight", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		
		// P-51D cruise conditions
		state := NewAircraftState()
		state.Velocity = Vector3{X: 120.0, Y: 0.0, Z: -2.0}  // 120 m/s forward, 2 m/s climb
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		// Gentle constant acceleration forward
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 0.1, Y: 0.0, Z: 0.0}, // Small forward acceleration
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		}
		
		dt := 0.01
		steps := 1000 // 10 seconds of flight
		
		for i := 0; i < steps; i++ {
			state = integrator.Integrate(state, derivatives, dt)
		}
		
		// Check that aircraft moved forward
		if state.Position.X <= 0 {
			t.Error("Aircraft should have moved forward")
		}
		
		// Check that speed increased
		if state.Velocity.X <= 120.0 {
			t.Error("Forward velocity should have increased")
		}
		
		t.Logf("Final position: (%.1f, %.1f, %.1f) m", 
			state.Position.X, state.Position.Y, state.Position.Z)
		t.Logf("Final velocity: %.1f m/s forward", state.Velocity.X)
	})
	
	t.Run("Banking Turn", func(t *testing.T) {
		integrator := NewRungeKutta4Integrator()
		
		state := NewAircraftState()
		state.Velocity = Vector3{X: 80.0, Y: 0.0, Z: 0.0}
		
		// Constant turn rate
		turnRate := 0.5 // rad/s
		
		// Set initial angular rate for turn
		state.AngularRate = Vector3{X: 0.0, Y: 0.0, Z: turnRate}
		
		derivatives := &StateDerivatives{
			VelocityDot:    Vector3{X: 0.0, Y: 0.0, Z: 0.0},
			AngularRateDot: Vector3{X: 0.0, Y: 0.0, Z: 0.0}, // Constant turn rate
		}
		
		dt := 0.01
		steps := 628 // About π seconds for half turn (π/0.5 * 100 steps/sec)
		
		initialHeading := state.Yaw
		
		for i := 0; i < steps; i++ {
			state = integrator.Integrate(state, derivatives, dt)
		}
		
		finalHeading := state.Yaw
		headingChange := math.Abs(finalHeading - initialHeading)
		
		// Should have turned approximately π radians (180°)
		assertApproxEqual(t, headingChange, math.Pi, 0.3)
		
		t.Logf("Heading change: %.1f° (expected ~180°)", headingChange*RAD_TO_DEG)
	})
}
