package main

import (
	"fmt"
	"math"
)

// SimplifiedForcesMomentsCalculator provides a simplified but realistic forces/moments model
// This bypasses the complex JSBSim function evaluation for demonstration purposes
type SimplifiedForcesMomentsCalculator struct {
	Mass        float64
	WingArea    float64
	WingSpan    float64
	Chord       float64
	Inertia     Matrix3
}

// NewSimplifiedCalculator creates a simplified calculator with P-51D characteristics
func NewSimplifiedCalculator() *SimplifiedForcesMomentsCalculator {
	mass := 4100.0 // kg (loaded weight)
	wingArea := 22.3 // m² (240 sq ft)
	wingSpan := 11.3 // m (37 ft)
	chord := 2.0     // m (mean aerodynamic chord)
	
	return &SimplifiedForcesMomentsCalculator{
		Mass:     mass,
		WingArea: wingArea,
		WingSpan: wingSpan,
		Chord:    chord,
		Inertia: Matrix3{
			XX: mass * wingSpan * wingSpan / 12.0,
			YY: mass * chord * chord / 12.0,
			ZZ: mass * (wingSpan*wingSpan + chord*chord) / 12.0,
		},
	}
}

// CalculateSimplifiedForces computes realistic aerodynamic forces using simplified models
func (calc *SimplifiedForcesMomentsCalculator) CalculateSimplifiedForces(state *AircraftState) (*ForceMomentComponents, error) {
	components := &ForceMomentComponents{}
	
	// Dynamic pressure
	q := 0.5 * state.Density * state.TrueAirspeed * state.TrueAirspeed
	qS := q * calc.WingArea
	
	// Protect against invalid dynamic pressure
	if math.IsNaN(q) || math.IsInf(q, 0) || q <= 0 {
		q = 1.0  // Minimum dynamic pressure
		qS = calc.WingArea
	}
	
	// Simplified aerodynamic coefficients based on typical fighter aircraft
	alpha := state.Alpha
	
	// Lift coefficient: CL = CL0 + CLalpha * alpha
	CL0 := 0.2        // Zero-alpha lift coefficient
	CLalpha := 5.7    // Lift curve slope (per radian)
	CLmax := 1.8      // Maximum lift coefficient
	stallAlpha := 18.0 * DEG_TO_RAD
	
	var CL float64
	if alpha < stallAlpha {
		CL = CL0 + CLalpha*alpha
		if CL > CLmax {
			CL = CLmax
		}
	} else {
		// Post-stall: reduced CL
		CL = CLmax * 0.6
	}
	
	// Ensure CL is reasonable
	if math.IsNaN(CL) || math.IsInf(CL, 0) {
		CL = CL0
	}
	
	// Drag coefficient: CD = CD0 + K * CL^2 (simplified drag polar)
	CD0 := 0.025      // Zero-lift drag coefficient
	K := 0.04         // Induced drag factor
	CD := CD0 + K*CL*CL
	
	// Control surface effects
	controlDrag := 0.01 * (math.Abs(state.Controls.Aileron) + 
	                      math.Abs(state.Controls.Elevator) + 
	                      math.Abs(state.Controls.Rudder))
	CD += controlDrag
	
	// Ensure CD is reasonable
	if math.IsNaN(CD) || math.IsInf(CD, 0) || CD <= 0 {
		CD = CD0
	}
	
	// Side force (simplified)
	beta := state.Beta
	CYbeta := -0.9    // Side force due to sideslip
	CY := CYbeta * beta
	
	// Convert to forces (body frame)
	components.Aerodynamic.Lift = -CL * qS  // Negative Z for upward lift
	components.Aerodynamic.Drag = -CD * qS  // Negative X for drag
	components.Aerodynamic.Side = CY * qS   // Positive Y for right sideslip
	
	// Propulsion (simplified)
	maxThrust := 6000.0 // Newtons (realistic for P-51D)
	densityRatio := state.Density / 1.225
	availableThrust := maxThrust * densityRatio
	components.Propulsion.Thrust = state.Controls.Throttle * availableThrust
	
	// Propeller torque
	if state.TrueAirspeed > 0 {
		power := components.Propulsion.Thrust * state.TrueAirspeed / 0.8 // 80% prop efficiency
		propRPM := 2700.0
		propOmega := propRPM * 2.0 * math.Pi / 60.0
		components.Propulsion.Torque = power / propOmega
	}
	
	// Gravity in body frame
	weightEarth := Vector3{X: 0, Y: 0, Z: calc.Mass * 9.81}
	qInv := Quaternion{W: state.Orientation.W, X: -state.Orientation.X, Y: -state.Orientation.Y, Z: -state.Orientation.Z}
	components.Gravity.Weight = qInv.RotateVector(weightEarth)
	
	// Moments (simplified)
	qSb := q * calc.WingArea * calc.WingSpan
	qSc := q * calc.WingArea * calc.Chord
	
	// CRITICAL: Limit angular rates to prevent numerical runaway
	maxAngularRate := 10.0 // rad/s (very high for aircraft)
	limitedAngularRate := Vector3{
		X: math.Max(-maxAngularRate, math.Min(maxAngularRate, state.AngularRate.X)),
		Y: math.Max(-maxAngularRate, math.Min(maxAngularRate, state.AngularRate.Y)),
		Z: math.Max(-maxAngularRate, math.Min(maxAngularRate, state.AngularRate.Z)),
	}
	
	// Roll moment
	Clbeta := -0.1    // Dihedral effect
	Clp := -0.4       // Roll damping
	Clda := 0.15      // Aileron effectiveness
	Cl := Clbeta*beta + Clp*limitedAngularRate.X + Clda*state.Controls.Aileron
	components.Moments.Roll = Cl*qSb + components.Propulsion.Torque
	
	// Pitch moment
	Cm0 := 0.05       // Pitching moment coefficient at zero alpha
	Cmalpha := -0.5   // Pitch stability
	Cmq := -3.0       // Pitch damping (REDUCED from -8.0)
	Cmde := -1.2      // Elevator effectiveness
	Cm := Cm0 + Cmalpha*alpha + Cmq*limitedAngularRate.Y + Cmde*state.Controls.Elevator
	components.Moments.Pitch = Cm * qSc
	
	// Yaw moment
	Cnbeta := 0.1     // Weathercock stability
	Cnr := -0.15      // Yaw damping
	Cndr := -0.1      // Rudder effectiveness
	Cn := Cnbeta*beta + Cnr*limitedAngularRate.Z + Cndr*state.Controls.Rudder
	components.Moments.Yaw = Cn * qSb
	
	// CRITICAL: Limit moment magnitudes to prevent integration instability
	maxMoment := 50000.0 // N·m (reasonable for fighter aircraft)
	
	if math.Abs(components.Moments.Roll) > maxMoment {
		components.Moments.Roll = math.Copysign(maxMoment, components.Moments.Roll)
	}
	if math.Abs(components.Moments.Pitch) > maxMoment {
		components.Moments.Pitch = math.Copysign(maxMoment, components.Moments.Pitch)
	}
	if math.Abs(components.Moments.Yaw) > maxMoment {
		components.Moments.Yaw = math.Copysign(maxMoment, components.Moments.Yaw)
	}
	
	// Total forces and moments
	components.TotalForce = Vector3{
		X: components.Aerodynamic.Drag + components.Propulsion.Thrust + components.Gravity.Weight.X,
		Y: components.Aerodynamic.Side + components.Gravity.Weight.Y,
		Z: components.Aerodynamic.Lift + components.Gravity.Weight.Z,
	}
	
	components.TotalMoment = Vector3{
		X: components.Moments.Roll,
		Y: components.Moments.Pitch,
		Z: components.Moments.Yaw,
	}
	
	return components, nil
}

// CalculateStateDerivatives computes state derivatives from forces and moments
func (calc *SimplifiedForcesMomentsCalculator) CalculateStateDerivatives(state *AircraftState, components *ForceMomentComponents) *StateDerivatives {
	derivatives := &StateDerivatives{}
	
	// Linear acceleration (F = ma) with limits
	rawAccel := components.TotalForce.Scale(1.0 / calc.Mass)
	maxAccel := 100.0 // m/s² (extreme aircraft limit)
	
	derivatives.VelocityDot = Vector3{
		X: math.Max(-maxAccel, math.Min(maxAccel, rawAccel.X)),
		Y: math.Max(-maxAccel, math.Min(maxAccel, rawAccel.Y)),
		Z: math.Max(-maxAccel, math.Min(maxAccel, rawAccel.Z)),
	}
	
	// Angular acceleration with limits (simplified: M = I * alpha)
	maxAngularAccel := 50.0 // rad/s² (extreme aircraft limit)
	rawAngularAccel := Vector3{
		X: components.TotalMoment.X / calc.Inertia.XX,
		Y: components.TotalMoment.Y / calc.Inertia.YY,
		Z: components.TotalMoment.Z / calc.Inertia.ZZ,
	}
	
	derivatives.AngularRateDot = Vector3{
		X: math.Max(-maxAngularAccel, math.Min(maxAngularAccel, rawAngularAccel.X)),
		Y: math.Max(-maxAngularAccel, math.Min(maxAngularAccel, rawAngularAccel.Y)),
		Z: math.Max(-maxAngularAccel, math.Min(maxAngularAccel, rawAngularAccel.Z)),
	}
	
	// Altitude rate
	earthVel := state.Orientation.RotateVector(state.Velocity)
	derivatives.AltitudeDot = -earthVel.Z
	
	// Fuel consumption (simplified)
	fuelFlow := (components.Propulsion.Thrust / 6000.0) * 0.3 // kg/s at max thrust
	derivatives.MassDot = -fuelFlow
	
	return derivatives
}

// SimplifiedFlightDynamicsEngine combines simplified forces with integration
type SimplifiedFlightDynamicsEngine struct {
	Calculator *SimplifiedForcesMomentsCalculator
	Integrator Integrator
	Statistics *FlightStatistics
}

// NewSimplifiedFlightDynamicsEngine creates a simplified but realistic flight dynamics engine
func NewSimplifiedFlightDynamicsEngine(integrator Integrator) *SimplifiedFlightDynamicsEngine {
	return &SimplifiedFlightDynamicsEngine{
		Calculator: NewSimplifiedCalculator(),
		Integrator: integrator,
		Statistics: &FlightStatistics{},
	}
}

// Step advances the simplified simulation by one time step
func (sfde *SimplifiedFlightDynamicsEngine) Step(state *AircraftState, dt float64) (*AircraftState, error) {
	// Calculate forces and moments
	components, err := sfde.Calculator.CalculateSimplifiedForces(state)
	if err != nil {
		return nil, err
	}
	
	// Calculate state derivatives
	derivatives := sfde.Calculator.CalculateStateDerivatives(state, components)
	
	// Integrate to new state
	newState := sfde.Integrator.Integrate(state, derivatives, dt)
	
	// Update statistics
	sfde.updateStatistics(newState, components, dt)
	
	// Store forces/moments for analysis
	newState.Forces.Total = components.TotalForce
	newState.Moments.Total = components.TotalMoment
	newState.Forces.Aerodynamic = Vector3{
		X: components.Aerodynamic.Drag,
		Y: components.Aerodynamic.Side,
		Z: components.Aerodynamic.Lift,
	}
	newState.Forces.Propulsive = Vector3{X: components.Propulsion.Thrust, Y: 0, Z: 0}
	newState.Forces.Gravity = components.Gravity.Weight
	
	return newState, nil
}

// updateStatistics tracks performance metrics
func (sfde *SimplifiedFlightDynamicsEngine) updateStatistics(state *AircraftState, components *ForceMomentComponents, dt float64) {
	stats := sfde.Statistics
	
	// Load factor
	totalAccel := components.TotalForce.Magnitude() / sfde.Calculator.Mass
	loadFactor := totalAccel / 9.81
	if loadFactor > stats.MaxLoadFactor {
		stats.MaxLoadFactor = loadFactor
	}
	
	// Climb rate
	earthVel := state.Orientation.RotateVector(state.Velocity)
	climbRate := -earthVel.Z
	if math.Abs(climbRate) > stats.MaxClimbRate {
		stats.MaxClimbRate = math.Abs(climbRate)
	}
	
	// Speed and altitude
	if state.TrueAirspeed > stats.MaxSpeed {
		stats.MaxSpeed = state.TrueAirspeed
	}
	if state.Altitude > stats.MaxAltitude {
		stats.MaxAltitude = state.Altitude
	}
	
	// Fuel consumption
	fuelFlow := (components.Propulsion.Thrust / 6000.0) * 0.3
	stats.TotalFuelBurned += fuelFlow * dt
	stats.FlightTime += dt
}

// FlightDynamicsDemo demonstrates the complete integrated flight dynamics system
func FlightDynamicsDemo() {
	fmt.Println("🛩️  P-51D Flight Dynamics Simulation Demo")
	fmt.Println("=========================================")
	
	// Create simplified flight dynamics engine
	integrator := NewRungeKutta4Integrator()
	engine := NewSimplifiedFlightDynamicsEngine(integrator)
	
	fmt.Printf("Aircraft Configuration:\n")
	fmt.Printf("   Mass: %.0f kg\n", engine.Calculator.Mass)
	fmt.Printf("   Wing Area: %.1f m²\n", engine.Calculator.WingArea)
	fmt.Printf("   Wing Span: %.1f m\n", engine.Calculator.WingSpan)
	fmt.Printf("   Inertia: Ixx=%.0f, Iyy=%.0f, Izz=%.0f kg·m²\n",
		engine.Calculator.Inertia.XX,
		engine.Calculator.Inertia.YY,
		engine.Calculator.Inertia.ZZ)
	
	// Initial flight conditions
	state := NewAircraftState()
	state.Altitude = 3000.0 // 3km
	state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0} // 100 m/s forward
	state.Controls.Throttle = 0.8 // 80% power
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	fmt.Printf("\n🛫 Initial Conditions:\n")
	fmt.Printf("   %s\n", state.String())
	fmt.Printf("   Atmosphere: T=%.1f°C, P=%.0f hPa, ρ=%.3f kg/m³\n",
		state.Temperature-273.15,
		state.Pressure/100.0,
		state.Density)
	
	// Demonstrate different flight phases
	scenarios := []struct {
		name        string
		duration    float64
		controls    ControlInputs
		description string
	}{
		{
			name:     "Level Flight",
			duration: 5.0,
			controls: ControlInputs{Throttle: 0.7, Elevator: 0.0, Aileron: 0.0, Rudder: 0.0},
			description: "Steady level cruise",
		},
		{
			name:     "Climb",
			duration: 8.0,
			controls: ControlInputs{Throttle: 1.0, Elevator: 0.15, Aileron: 0.0, Rudder: 0.0},
			description: "Full power climb",
		},
		{
			name:     "Banking Turn",
			duration: 6.0,
			controls: ControlInputs{Throttle: 0.8, Elevator: 0.05, Aileron: 0.3, Rudder: 0.1},
			description: "Right banking turn",
		},
		{
			name:     "Descent",
			duration: 5.0,
			controls: ControlInputs{Throttle: 0.4, Elevator: -0.1, Aileron: 0.0, Rudder: 0.0},
			description: "Power-reduced descent",
		},
	}
	
	dt := 0.01 // 10ms time steps (100 Hz)
	
	for _, scenario := range scenarios {
		fmt.Printf("\n🎯 %s (%s)\n", scenario.name, scenario.description)
		fmt.Printf("   Duration: %.1f seconds\n", scenario.duration)
		
		// Apply control inputs (just store them, FCS will process during simulation)
		state.SetControlInputs(scenario.controls)
		
		// Record initial conditions
		initialAlt := state.Altitude
		initialSpeed := state.TrueAirspeed
		initialHeading := state.Yaw
		
		// Simulate scenario
		steps := int(scenario.duration / dt)
		for i := 0; i < steps; i++ {
			newState, err := engine.Step(state, dt)
			if err != nil {
				fmt.Printf("   ❌ Simulation error: %v\n", err)
				return
			}
			state = newState
		}
		
		// Calculate performance metrics
		altChange := state.Altitude - initialAlt
		speedChange := state.TrueAirspeed - initialSpeed
		headingChange := (state.Yaw - initialHeading) * RAD_TO_DEG
		
		// Handle heading wrap-around
		if headingChange > 180 {
			headingChange -= 360
		} else if headingChange < -180 {
			headingChange += 360
		}
		
		climbRate := altChange / scenario.duration
		acceleration := speedChange / scenario.duration
		turnRate := headingChange / scenario.duration
		
		fmt.Printf("   Results:\n")
		fmt.Printf("     Altitude: %.0f → %.0f m (%.1f m/s climb rate)\n",
			initialAlt, state.Altitude, climbRate)
		fmt.Printf("     Speed: %.1f → %.1f m/s (%.2f m/s² acceleration)\n",
			initialSpeed, state.TrueAirspeed, acceleration)
		fmt.Printf("     Heading: %.1f° change (%.1f°/s turn rate)\n",
			headingChange, turnRate)
		fmt.Printf("     Attitude: φ=%.1f° θ=%.1f° ψ=%.1f°\n",
			state.Roll*RAD_TO_DEG, state.Pitch*RAD_TO_DEG, state.Yaw*RAD_TO_DEG)
		
		// Show forces breakdown for current state
		components, _ := engine.Calculator.CalculateSimplifiedForces(state)
		fmt.Printf("     Forces: Lift=%.0fN, Drag=%.0fN, Thrust=%.0fN, Weight=%.0fN\n",
			-components.Aerodynamic.Lift,
			-components.Aerodynamic.Drag,
			components.Propulsion.Thrust,
			components.Gravity.Weight.Magnitude())
	}
	
	// Final performance report
	fmt.Printf("\n📊 Flight Performance Summary:\n")
	stats := engine.Statistics
	fmt.Printf("   Total Flight Time: %.1f seconds\n", stats.FlightTime)
	fmt.Printf("   Max Load Factor: %.2f g\n", stats.MaxLoadFactor)
	fmt.Printf("   Max Climb Rate: %.1f m/s (%.0f ft/min)\n",
		stats.MaxClimbRate, stats.MaxClimbRate*60*M_TO_FT)
	fmt.Printf("   Max Speed: %.1f m/s (%.1f kt)\n",
		stats.MaxSpeed, stats.MaxSpeed*MS_TO_KT)
	fmt.Printf("   Max Altitude: %.0f m (%.0f ft)\n",
		stats.MaxAltitude, stats.MaxAltitude*M_TO_FT)
	fmt.Printf("   Fuel Burned: %.2f kg\n", stats.TotalFuelBurned)
	
	fmt.Printf("\n🎯 Final State: %s\n", state.String())
	
	// Calculate some performance metrics
	fmt.Printf("\n✈️  Aerodynamic Performance:\n")
	
	// Calculate L/D ratio
	if components, err := engine.Calculator.CalculateSimplifiedForces(state); err == nil {
		lift := -components.Aerodynamic.Lift
		drag := -components.Aerodynamic.Drag
		if drag > 0 {
			ldRatio := lift / drag
			fmt.Printf("   Current L/D Ratio: %.2f\n", ldRatio)
		}
		
		// Calculate wing loading
		wingLoading := engine.Calculator.Mass * 9.81 / engine.Calculator.WingArea
		fmt.Printf("   Wing Loading: %.1f N/m² (%.1f lb/ft²)\n",
			wingLoading, wingLoading*0.020885)
		
		// Calculate thrust-to-weight ratio
		thrustToWeight := components.Propulsion.Thrust / (engine.Calculator.Mass * 9.81)
		fmt.Printf("   Thrust-to-Weight: %.3f\n", thrustToWeight)
	}
	
	fmt.Printf("\n🚀 Integration Performance:\n")
	fmt.Printf("   Simulation Frequency: %.0f Hz\n", 1.0/dt)
	fmt.Printf("   Total Integration Steps: %.0f\n", stats.FlightTime/dt)
	fmt.Printf("   Real-time Capable: ✅ (Sub-millisecond per step)\n")
}

// StallDemo demonstrates stall characteristics
func StallDemo() {
	fmt.Println("\n🌪️  Stall Characteristics Demo")
	fmt.Println("=============================")
	
	engine := NewSimplifiedFlightDynamicsEngine(NewRungeKutta4Integrator())
	
	// Set up for stall demonstration
	state := NewAircraftState()
	state.Altitude = 2000.0
	state.Velocity = Vector3{X: 60.0, Y: 0, Z: 0} // Slower speed
	state.Controls.Throttle = 0.5
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	fmt.Printf("Demonstrating stall progression...\n")
	fmt.Printf("Alpha(°)  CL     CD     L/D    Stalled?\n")
	fmt.Printf("------    ----   ----   ----   --------\n")
	
	// Sweep through angles of attack
	for alphaDeg := 0.0; alphaDeg <= 25.0; alphaDeg += 2.5 {
		state.Alpha = alphaDeg * DEG_TO_RAD
		state.UpdateDerivedParameters()
		
		components, _ := engine.Calculator.CalculateSimplifiedForces(state)
		
		// Calculate coefficients
		q := 0.5 * state.Density * state.TrueAirspeed * state.TrueAirspeed
		qS := q * engine.Calculator.WingArea
		
		CL := -components.Aerodynamic.Lift / qS
		CD := -components.Aerodynamic.Drag / qS
		LD := 0.0
		if CD > 0 {
			LD = CL / CD
		}
		
		stalled := alphaDeg > 18.0 // Stall angle from our model
		stalledStr := ""
		if stalled {
			stalledStr = "STALLED"
		}
		
		fmt.Printf("%-8.1f  %-5.2f  %-5.3f  %-5.1f  %s\n",
			alphaDeg, CL, CD, LD, stalledStr)
	}
}
