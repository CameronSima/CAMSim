// Propulsion System Integration Demo
// Demonstrates the complete flight dynamics system with propulsion

package main

import (
	"fmt"
	"math"
	"os"
	"strings"
)

// DemoIntegratedFlightDynamics demonstrates the complete integrated system
func DemoIntegratedFlightDynamics() {
	fmt.Println("\n" + strings.Repeat("=", 80))
	fmt.Println("🚁 P-51D INTEGRATED FLIGHT DYNAMICS DEMONSTRATION")
	fmt.Println(strings.Repeat("=", 80))
	
	// Load P-51D configuration
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		fmt.Printf("❌ Error loading P-51D config: %v\n", err)
		return
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		fmt.Printf("❌ Error parsing P-51D config: %v\n", err)
		return
	}
	
	// Create integrated flight dynamics engine
	fmt.Println("🔧 Creating integrated flight dynamics engine...")
	engine, err := NewFlightDynamicsEngineWithPropulsion(config, true, true)
	if err != nil {
		fmt.Printf("❌ Error creating engine: %v\n", err)
		return
	}
	
	fmt.Println("✅ Integrated system ready!")
	fmt.Printf("   - Flight Control System: %s\n", boolToStatus(engine.UseRealisticControls))
	fmt.Printf("   - Propulsion System: %s\n", boolToStatus(engine.UseRealisticPropulsion))
	fmt.Printf("   - Initial Fuel: %.1f lbs\n", engine.LastFuelWeight)
	
	// Demonstrate engine startup sequence
	demoEngineStartup(engine)
	
	// Demonstrate throttle response
	demoThrottleResponse(engine)
	
	// Demonstrate takeoff simulation
	demoTakeoffSimulation(engine)
	
	// Demonstrate fuel consumption
	demoFuelConsumption(engine)
	
	fmt.Println("\n" + strings.Repeat("=", 80))
	fmt.Println("✅ INTEGRATED FLIGHT DYNAMICS DEMONSTRATION COMPLETE")
	fmt.Println(strings.Repeat("=", 80))
}

// demoEngineStartup demonstrates the engine startup sequence
func demoEngineStartup(engine *FlightDynamicsEngineWithPropulsion) {
	fmt.Println("\n🔥 ENGINE STARTUP SEQUENCE")
	fmt.Println(strings.Repeat("-", 50))
	
	// Create initial state (on ground, engine off)
	state := &AircraftState{
		Position:        Vector3{X: 0, Y: 0, Z: 0}, // On ground
		Velocity:        Vector3{X: 0, Y: 0, Z: 0}, // Stationary
		Orientation:     Quaternion{W: 1, X: 0, Y: 0, Z: 0}, // Level
		AngularRate:     Vector3{X: 0, Y: 0, Z: 0},
		Controls:        ControlInputs{},
	}
	
	// Engine initially off
	fmt.Println("Engine Status: OFF")
	fmt.Printf("  RPM: %.0f\n", engine.Propulsion.Engine.RPM)
	fmt.Printf("  MAP: %.1f inHg\n", engine.Propulsion.Engine.ManifoldPressure)
	fmt.Printf("  Thrust: %.1f lbs\n", engine.Propulsion.Propeller.Thrust)
	
	// Apply throttle to start engine
	fmt.Println("\n🎛️  Applying 20% throttle to start engine...")
	state.Controls.Throttle = 0.2
	
	// Run a few simulation steps
	for i := 0; i < 5; i++ {
		newState, _, err := engine.RunSimulationStepWithPropulsion(state, 0.1)
		if err != nil {
			fmt.Printf("❌ Simulation error: %v\n", err)
			return
		}
		state = newState
	}
	
	fmt.Println("Engine Status: RUNNING")
	fmt.Printf("  RPM: %.0f\n", engine.Propulsion.Engine.RPM)
	fmt.Printf("  MAP: %.1f inHg\n", engine.Propulsion.Engine.ManifoldPressure)
	fmt.Printf("  Thrust: %.1f lbs\n", engine.Propulsion.Propeller.Thrust)
	fmt.Printf("  Fuel Flow: %.1f lbs/hr\n", engine.Propulsion.FuelSystem.FuelFlow)
}

// demoThrottleResponse demonstrates throttle response characteristics
func demoThrottleResponse(engine *FlightDynamicsEngineWithPropulsion) {
	fmt.Println("\n🎛️  THROTTLE RESPONSE CHARACTERISTICS")
	fmt.Println(strings.Repeat("-", 50))
	
	state := &AircraftState{
		Position:        Vector3{X: 0, Y: 0, Z: -1000}, // 1000m altitude
		Velocity:        Vector3{X: 60, Y: 0, Z: 0},    // 60 m/s cruise
		Orientation:     Quaternion{W: 1, X: 0, Y: 0, Z: 0},
		AngularRate:     Vector3{X: 0, Y: 0, Z: 0},
		Controls:        ControlInputs{},
	}
	
	fmt.Println("Throttle  RPM    MAP     Thrust   T/W     Accel   Fuel Flow")
	fmt.Println("--------  ----   -----   ------   ----    -----   ---------")
	
	throttleSettings := []float64{0.2, 0.4, 0.6, 0.8, 1.0}
	for _, throttle := range throttleSettings {
		state.Controls.Throttle = throttle
		
		// Run simulation step
		newState, derivatives, err := engine.RunSimulationStepWithPropulsion(state, 0.01)
		if err != nil {
			fmt.Printf("❌ Error at throttle %.1f: %v\n", throttle, err)
			continue
		}
		
		twr := engine.GetThrustToWeightRatio()
		accel := derivatives.VelocityDot.X
		
		fmt.Printf("%6.0f%%  %4.0f   %5.1f   %6.1f   %4.3f   %5.2f   %7.1f",
			throttle*100,
			engine.Propulsion.Engine.RPM,
			engine.Propulsion.Engine.ManifoldPressure,
			engine.Propulsion.Propeller.Thrust,
			twr,
			accel,
			engine.Propulsion.FuelSystem.FuelFlow)
		fmt.Println()
		
		state = newState
	}
}

// demoTakeoffSimulation demonstrates a takeoff sequence
func demoTakeoffSimulation(engine *FlightDynamicsEngineWithPropulsion) {
	fmt.Println("\n🛫 TAKEOFF SIMULATION")
	fmt.Println(strings.Repeat("-", 50))
	
	// Start on runway
	state := &AircraftState{
		Position:        Vector3{X: 0, Y: 0, Z: 0}, // Ground level
		Velocity:        Vector3{X: 0, Y: 0, Z: 0}, // Stationary
		Orientation:     Quaternion{W: 1, X: 0, Y: 0, Z: 0}, // Level
		AngularRate:     Vector3{X: 0, Y: 0, Z: 0},
		Controls:        ControlInputs{Throttle: 1.0}, // Full throttle
	}
	
	fmt.Println("Time   Speed    Altitude  Thrust   RPM    Fuel")
	fmt.Println("(s)    (kt)     (ft)      (lbs)    (rpm)  (lbs)")
	fmt.Println("----   -----    --------  -----    ----   ----")
	
	totalTime := 0.0
	dt := 0.5 // 0.5 second time steps
	
	for i := 0; i < 60; i++ { // 30 seconds of simulation
		newState, _, err := engine.RunSimulationStepWithPropulsion(state, dt)
		if err != nil {
			fmt.Printf("❌ Simulation error at t=%.1f: %v\n", totalTime, err)
			break
		}
		
		totalTime += dt
		
		// Calculate display values
		speed := math.Sqrt(newState.Velocity.X*newState.Velocity.X + 
		                  newState.Velocity.Y*newState.Velocity.Y) * 1.944 // m/s to knots
		altitude := -newState.Position.Z * 3.28084 // meters to feet (negative Z is up)
		
		// Print status every 2.5 seconds
		if i%5 == 0 {
			fmt.Printf("%4.1f   %5.1f    %8.0f  %5.1f    %4.0f   %4.0f\n",
				totalTime,
				speed,
				altitude,
				engine.Propulsion.Propeller.Thrust,
				engine.Propulsion.Engine.RPM,
				engine.Propulsion.FuelSystem.TotalContents)
		}
		
		state = newState
		
		// Check for rotation speed (typical P-51D: ~100 kt)
		if speed > 100 && totalTime < 20 {
			fmt.Printf("     🛫 Rotation speed reached at t=%.1f s (%.1f kt)\n", totalTime, speed)
		}
		
		// Check for liftoff
		if altitude > 10 && totalTime < 25 {
			fmt.Printf("     ✈️  Liftoff at t=%.1f s (%.0f ft altitude)\n", totalTime, altitude)
		}
	}
	
	finalSpeed := math.Sqrt(state.Velocity.X*state.Velocity.X + 
	                       state.Velocity.Y*state.Velocity.Y) * 1.944
	finalAltitude := -state.Position.Z * 3.28084
	
	fmt.Printf("\nTakeoff Results:\n")
	fmt.Printf("  Final Speed: %.1f kt\n", finalSpeed)
	fmt.Printf("  Final Altitude: %.0f ft\n", finalAltitude)
	fmt.Printf("  Fuel Remaining: %.1f lbs\n", engine.Propulsion.FuelSystem.TotalContents)
}

// demoFuelConsumption demonstrates fuel consumption over time
func demoFuelConsumption(engine *FlightDynamicsEngineWithPropulsion) {
	fmt.Println("\n⛽ FUEL CONSUMPTION ANALYSIS")
	fmt.Println(strings.Repeat("-", 50))
	
	// Cruise flight state
	state := &AircraftState{
		Position:        Vector3{X: 0, Y: 0, Z: -3000}, // 3000m altitude
		Velocity:        Vector3{X: 100, Y: 0, Z: 0},   // 100 m/s cruise
		Orientation:     Quaternion{W: 1, X: 0, Y: 0, Z: 0},
		AngularRate:     Vector3{X: 0, Y: 0, Z: 0},
		Controls:        ControlInputs{Throttle: 0.65}, // 65% cruise power
	}
	
	initialFuel := engine.Propulsion.FuelSystem.TotalContents
	approxWeight := 10000.0 // Approximate cruise weight in lbs
	
	fmt.Printf("Initial Conditions:\n")
	fmt.Printf("  Fuel: %.1f lbs\n", initialFuel)
	fmt.Printf("  Weight: %.0f lbs (approx)\n", approxWeight)
	fmt.Printf("  Throttle: %.0f%%\n", state.Controls.Throttle*100)
	fmt.Printf("  Speed: %.1f kt\n", state.Velocity.X*1.944)
	fmt.Printf("  Altitude: %.0f ft\n", -state.Position.Z*3.28084)
	
	fmt.Println("\nTime    Fuel     Weight   Flow     Range    Endurance")
	fmt.Println("(min)   (lbs)    (lbs)    (lb/hr)  (nm)     (hr)")
	fmt.Println("-----   -----    -----    ------   -----    ---------")
	
	totalTime := 0.0
	dt := 60.0 // 1-minute time steps
	distance := 0.0
	
	for i := 0; i <= 180; i++ { // 3 hours of simulation
		if i > 0 {
			newState, _, err := engine.RunSimulationStepWithPropulsion(state, dt)
			if err != nil {
				fmt.Printf("❌ Simulation error at t=%.0f min: %v\n", totalTime, err)
				break
			}
			
			// Update distance traveled
			distance += state.Velocity.X * dt / 1852.0 // Convert to nautical miles
			state = newState
		}
		
		totalTime += dt / 60.0 // Convert to minutes
		
		currentFuel := engine.Propulsion.FuelSystem.TotalContents
		fuelBurned := initialFuel - currentFuel
		currentWeight := approxWeight - fuelBurned // Approximate current weight
		fuelFlow := engine.Propulsion.FuelSystem.FuelFlow
		
		// Calculate endurance based on current fuel flow
		endurance := 0.0
		if fuelFlow > 0 {
			endurance = currentFuel / fuelFlow
		}
		
		// Print status every 30 minutes
		if i%30 == 0 {
			fmt.Printf("%5.0f   %5.0f    %5.0f    %6.1f   %5.0f    %7.1f\n",
				totalTime,
				currentFuel,
				currentWeight,
				fuelFlow,
				distance,
				endurance)
		}
		
		// Check if fuel is getting low
		if currentFuel < initialFuel*0.1 && currentFuel > 0 {
			fmt.Printf("     ⚠️  Low fuel warning: %.0f lbs remaining\n", currentFuel)
			break
		}
		
		if currentFuel <= 0 {
			fmt.Printf("     ⛽ Fuel exhausted at t=%.0f minutes\n", totalTime)
			break
		}
	}
	
	fuelBurned := initialFuel - engine.Propulsion.FuelSystem.TotalContents
	weightLoss := fuelBurned // Approximate weight loss equals fuel burned
	
	fmt.Printf("\nFuel Consumption Summary:\n")
	fmt.Printf("  Total Time: %.1f hours\n", totalTime/60.0)
	fmt.Printf("  Fuel Burned: %.1f lbs (%.1f gal)\n", fuelBurned, fuelBurned/6.0)
	fmt.Printf("  Weight Loss: %.1f lbs\n", weightLoss)
	fmt.Printf("  Average Flow: %.1f lbs/hr\n", fuelBurned/(totalTime/60.0))
	fmt.Printf("  Distance: %.0f nm\n", distance)
	fmt.Printf("  Fuel Economy: %.2f nm/gal\n", distance/(fuelBurned/6.0))
}

// Helper function to convert bool to status string
func boolToStatus(b bool) string {
	if b {
		return "Realistic/Advanced"
	}
	return "Simplified/Basic"
}
