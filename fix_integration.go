package main

import (
	"fmt"
	"math"
)

// FixIntegrationDemo demonstrates the corrected integration setup
func FixIntegrationDemo() {
	fmt.Println("🔧 Testing Fixed Integration Setup")
	fmt.Println("==================================")
	
	engine := NewSimplifiedFlightDynamicsEngine(NewEulerIntegrator())
	
	// Create initial state with PROPER position/altitude coordination
	state := NewAircraftState()
	
	// Set desired altitude and COORDINATE with position
	desiredAltitude := 3000.0
	state.Altitude = desiredAltitude
	state.Position.Z = -desiredAltitude  // NED frame: negative Z for positive altitude
	
	// Set initial velocity
	state.Velocity = Vector3{X: 100.0, Y: 0, Z: 0}
	state.Controls.Throttle = 0.7
	
	// Update atmosphere and derived parameters
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	fmt.Printf("✅ Properly initialized state:\n")
	fmt.Printf("   Altitude: %.1f m\n", state.Altitude)
	fmt.Printf("   Position.Z: %.1f m\n", state.Position.Z)
	fmt.Printf("   Velocity: (%.1f, %.1f, %.1f) m/s\n", state.Velocity.X, state.Velocity.Y, state.Velocity.Z)
	fmt.Printf("   Atmosphere: T=%.1f°C, ρ=%.3f kg/m³\n", state.Temperature-273.15, state.Density)
	
	// Run simulation for 5 seconds
	dt := 0.01
	steps := 500
	
	fmt.Printf("\n🛫 Running 5-second simulation...\n")
	
	initialAlt := state.Altitude
	initialSpeed := state.TrueAirspeed
	
	for i := 0; i < steps; i++ {
		newState, err := engine.Step(state, dt)
		if err != nil {
			fmt.Printf("❌ Error at step %d: %v\n", i+1, err)
			return
		}
		
		// Check for NaN values
		if hasNaNValues(newState) {
			fmt.Printf("❌ NaN detected at step %d\n", i+1)
			printStateDebug(newState, "NaN_STATE")
			return
		}
		
		state = newState
		
		// Print progress every second
		if (i+1)%100 == 0 {
			fmt.Printf("   t=%.1fs: Alt=%.1fm, Speed=%.1fm/s, Pos=(%.1f,%.1f,%.1f)\n",
				state.Time, state.Altitude, state.TrueAirspeed,
				state.Position.X, state.Position.Y, state.Position.Z)
		}
	}
	
	// Final results
	fmt.Printf("\n📊 Final Results:\n")
	fmt.Printf("   Flight Time: %.1f seconds\n", state.Time)
	fmt.Printf("   Altitude: %.1f → %.1f m (change: %.1f m)\n", 
		initialAlt, state.Altitude, state.Altitude-initialAlt)
	fmt.Printf("   Speed: %.1f → %.1f m/s (change: %.1f m/s)\n",
		initialSpeed, state.TrueAirspeed, state.TrueAirspeed-initialSpeed)
	fmt.Printf("   Distance: %.1f m\n", state.Position.X)
	fmt.Printf("   Final Attitude: φ=%.1f° θ=%.1f° ψ=%.1f°\n",
		state.Roll*RAD_TO_DEG, state.Pitch*RAD_TO_DEG, state.Yaw*RAD_TO_DEG)
	
	// Calculate current forces for analysis
	components, _ := engine.Calculator.CalculateSimplifiedForces(state)
	if components != nil {
		lift := -components.Aerodynamic.Lift
		drag := -components.Aerodynamic.Drag
		thrust := components.Propulsion.Thrust
		weight := components.Gravity.Weight.Magnitude()
		
		fmt.Printf("\n✈️  Current Forces:\n")
		fmt.Printf("   Lift: %.0f N\n", lift)
		fmt.Printf("   Drag: %.0f N\n", drag)
		fmt.Printf("   Thrust: %.0f N\n", thrust)
		fmt.Printf("   Weight: %.0f N\n", weight)
		
		if drag > 0 {
			ldRatio := lift / drag
			fmt.Printf("   L/D Ratio: %.2f\n", ldRatio)
		}
		
		// Check force balance
		netVertical := lift - weight
		netHorizontal := thrust - drag
		fmt.Printf("   Net Vertical Force: %.0f N (%.1f g)\n", netVertical, netVertical/(engine.Calculator.Mass*9.81))
		fmt.Printf("   Net Horizontal Force: %.0f N\n", netHorizontal)
	}
	
	if !hasNaNValues(state) {
		fmt.Printf("\n✅ Simulation completed successfully with no NaN values!\n")
	}
}

// TestClimbingFlight demonstrates a climbing flight scenario
func TestClimbingFlight() {
	fmt.Println("\n🚁 Testing Climbing Flight")
	fmt.Println("==========================")
	
	engine := NewSimplifiedFlightDynamicsEngine(NewRungeKutta4Integrator())
	
	// Start at lower altitude
	state := NewAircraftState()
	state.Altitude = 1000.0
	state.Position.Z = -1000.0  // Coordinate with altitude
	state.Velocity = Vector3{X: 80.0, Y: 0, Z: -3.0}  // 80 m/s forward, 3 m/s climb
	state.Controls.Throttle = 1.0    // Full power
	state.Controls.Elevator = 0.1    // Slight up elevator
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	fmt.Printf("🛫 Initial Climb Conditions:\n")
	fmt.Printf("   Altitude: %.0f m\n", state.Altitude)
	fmt.Printf("   Climb Rate: %.1f m/s\n", -state.Velocity.Z)
	fmt.Printf("   Forward Speed: %.1f m/s\n", state.Velocity.X)
	fmt.Printf("   Throttle: %.0f%%\n", state.Controls.Throttle*100)
	
	// Simulate climb for 30 seconds
	dt := 0.01
	steps := 3000
	
	initialAlt := state.Altitude
	maxAlt := state.Altitude
	maxClimbRate := 0.0
	
	for i := 0; i < steps; i++ {
		newState, err := engine.Step(state, dt)
		if err != nil {
			fmt.Printf("❌ Climb simulation error: %v\n", err)
			return
		}
		
		if hasNaNValues(newState) {
			fmt.Printf("❌ NaN in climb simulation at step %d\n", i+1)
			return
		}
		
		state = newState
		
		// Track performance
		if state.Altitude > maxAlt {
			maxAlt = state.Altitude
		}
		
		earthVel := state.Orientation.RotateVector(state.Velocity)
		climbRate := -earthVel.Z
		if climbRate > maxClimbRate {
			maxClimbRate = climbRate
		}
		
		// Progress every 5 seconds
		if (i+1)%(5*100) == 0 {
			fmt.Printf("   t=%ds: Alt=%.0fm, Climb=%.1fm/s, Speed=%.1fm/s\n",
				int(state.Time), state.Altitude, climbRate, state.TrueAirspeed)
		}
	}
	
	altGain := state.Altitude - initialAlt
	avgClimbRate := altGain / 30.0
	
	fmt.Printf("\n📈 Climb Performance:\n")
	fmt.Printf("   Altitude Gain: %.0f m (%.0f ft)\n", altGain, altGain*M_TO_FT)
	fmt.Printf("   Average Climb Rate: %.1f m/s (%.0f ft/min)\n", 
		avgClimbRate, avgClimbRate*60*M_TO_FT)
	fmt.Printf("   Max Climb Rate: %.1f m/s (%.0f ft/min)\n",
		maxClimbRate, maxClimbRate*60*M_TO_FT)
	fmt.Printf("   Final Altitude: %.0f m (%.0f ft)\n", 
		state.Altitude, state.Altitude*M_TO_FT)
	fmt.Printf("   Final Speed: %.1f m/s (%.1f kt)\n", 
		state.TrueAirspeed, state.TrueAirspeed*MS_TO_KT)
	
	if avgClimbRate > 0 {
		fmt.Printf("✅ Successful climb simulation!\n")
	} else {
		fmt.Printf("⚠️  Aircraft did not climb as expected\n")
	}
}

// TestBankingTurn demonstrates a coordinated banking turn
func TestBankingTurn() {
	fmt.Println("\n🔄 Testing Banking Turn")
	fmt.Println("=======================")
	
	engine := NewSimplifiedFlightDynamicsEngine(NewRungeKutta4Integrator())
	
	// Level flight conditions
	state := NewAircraftState()
	state.Altitude = 2000.0
	state.Position.Z = -2000.0
	state.Velocity = Vector3{X: 90.0, Y: 0, Z: 0}
	state.Controls.Throttle = 0.8
	state.Controls.Aileron = 0.3     // Bank input
	state.Controls.Rudder = 0.1      // Coordinated turn
	state.Controls.Elevator = 0.05   // Slight back pressure
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	fmt.Printf("🔄 Turn Setup:\n")
	fmt.Printf("   Initial Heading: %.1f°\n", state.Yaw*RAD_TO_DEG)
	fmt.Printf("   Speed: %.1f m/s\n", state.TrueAirspeed)
	fmt.Printf("   Control Inputs: Aileron=%.1f, Rudder=%.1f, Elevator=%.2f\n",
		state.Controls.Aileron, state.Controls.Rudder, state.Controls.Elevator)
	
	// Simulate 360° turn
	dt := 0.01
	steps := 2000 // 20 seconds
	
	initialHeading := state.Yaw
	initialAlt := state.Altitude
	maxBank := 0.0
	maxTurnRate := 0.0
	
	for i := 0; i < steps; i++ {
		newState, err := engine.Step(state, dt)
		if err != nil {
			fmt.Printf("❌ Turn simulation error: %v\n", err)
			return
		}
		
		if hasNaNValues(newState) {
			fmt.Printf("❌ NaN in turn simulation at step %d\n", i+1)
			return
		}
		
		state = newState
		
		// Track turn performance
		bankAngle := math.Abs(state.Roll)
		if bankAngle > maxBank {
			maxBank = bankAngle
		}
		
		turnRate := math.Abs(state.AngularRate.Z)
		if turnRate > maxTurnRate {
			maxTurnRate = turnRate
		}
		
		// Progress every 4 seconds
		if (i+1)%(4*100) == 0 {
			fmt.Printf("   t=%ds: Hdg=%.0f°, Bank=%.1f°, Alt=%.0fm\n",
				int(state.Time), state.Yaw*RAD_TO_DEG, state.Roll*RAD_TO_DEG, state.Altitude)
		}
	}
	
	finalHeading := state.Yaw
	headingChange := (finalHeading - initialHeading) * RAD_TO_DEG
	
	// Handle heading wrap-around
	if headingChange > 180 {
		headingChange -= 360
	} else if headingChange < -180 {
		headingChange += 360
	}
	
	altLoss := initialAlt - state.Altitude
	
	fmt.Printf("\n🎯 Turn Performance:\n")
	fmt.Printf("   Heading Change: %.1f°\n", headingChange)
	fmt.Printf("   Max Bank Angle: %.1f°\n", maxBank*RAD_TO_DEG)
	fmt.Printf("   Max Turn Rate: %.1f°/s\n", maxTurnRate*RAD_TO_DEG)
	fmt.Printf("   Altitude Loss: %.1f m\n", altLoss)
	fmt.Printf("   Final Speed: %.1f m/s\n", state.TrueAirspeed)
	
	if math.Abs(headingChange) > 90 {
		fmt.Printf("✅ Successful turning flight!\n")
	} else {
		fmt.Printf("⚠️  Turn performance less than expected\n")
	}
}
