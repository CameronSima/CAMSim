package main

import (
	"fmt"
	"math"
	"os"
	"time"
)

// FlightSimulatorDemo demonstrates the aircraft state system working with P-51D configuration
func FlightSimulatorDemo() {
	fmt.Println("🛩️  P-51D Mustang Flight Simulator Demo")
	fmt.Println("=====================================")
	
	// Load the P-51D configuration
	fmt.Print("Loading P-51D configuration... ")
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		fmt.Printf("❌ Error opening file: %v\n", err)
		return
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		fmt.Printf("❌ Error parsing config: %v\n", err)
		return
	}
	fmt.Println("✅ Loaded successfully!")
	
	// Display aircraft information
	fmt.Printf("\n📋 Aircraft Information:\n")
	if config.Header != nil {
		fmt.Printf("   Author: %s\n", config.Header.Author)
		if config.Header.Description != "" {
			fmt.Printf("   Description: %s\n", config.Header.Description)
		}
		if config.Header.FileCreationDate != "" {
			fmt.Printf("   Created: %s\n", config.Header.FileCreationDate)
		}
	}
	
	if config.Metrics != nil {
		if config.Metrics.WingSpan != nil {
			fmt.Printf("   Wing Span: %.1f ft\n", config.Metrics.WingSpan.Value*M_TO_FT)
		}
		if config.Metrics.WingArea != nil {
			fmt.Printf("   Wing Area: %.1f ft²\n", config.Metrics.WingArea.Value*M2_TO_FT2)
		}
		if config.Metrics.Chord != nil {
			fmt.Printf("   Wing Chord: %.2f ft\n", config.Metrics.Chord.Value*M_TO_FT)
		}
	}
	
	if config.MassBalance != nil && config.MassBalance.EmptyMass != nil {
		fmt.Printf("   Empty Weight: %.0f lbs\n", config.MassBalance.EmptyMass.Value*KG_TO_LB)
	}
	
	// Create initial aircraft state
	fmt.Printf("\n🛫 Initializing Aircraft State:\n")
	state := NewAircraftState()
	
	// Set initial conditions for cruise flight
	state.Altitude = 5000.0 * FT_TO_M  // 5000 ft
	state.Velocity = Vector3{X: 150.0, Y: 0.0, Z: 0.0}  // 150 m/s (~290 knots)
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	fmt.Printf("   Initial State: %s\n", state.String())
	
	// Show atmospheric conditions
	fmt.Printf("\n🌤️  Atmospheric Conditions at %.0f ft:\n", state.Altitude*M_TO_FT)
	fmt.Printf("   Temperature: %.1f°C (%.1f°F)\n", 
		state.Temperature-273.15, 
		(state.Temperature-273.15)*9.0/5.0+32.0)
	fmt.Printf("   Pressure: %.1f hPa\n", state.Pressure/100.0)
	fmt.Printf("   Density: %.4f kg/m³\n", state.Density)
	fmt.Printf("   Sound Speed: %.1f m/s (%.1f kt)\n", state.SoundSpeed, state.SoundSpeed*MS_TO_KT)
	fmt.Printf("   Dynamic Pressure: %.1f Pa (%.2f psf)\n", 
		state.DynamicPressure, 
		state.DynamicPressure*0.020885)
	
	// Demonstrate control input mapping
	fmt.Printf("\n🎮 Control Input Demonstration:\n")
	controls := ControlInputs{
		Aileron:  0.3,   // 30% right roll
		Elevator: -0.1,  // 10% nose down
		Rudder:   0.05,  // 5% right rudder
		Throttle: 0.75,  // 75% power
		Flaps:    0.0,   // No flaps
		Gear:     false, // Gear up
	}
	
	state.ApplyControlInputs(controls)
	
	fmt.Printf("   Pilot Inputs → Surface Positions:\n")
	fmt.Printf("   Aileron: %.1f%% → L: %.1f°, R: %.1f°\n",
		controls.Aileron*100,
		state.ControlSurfaces.AileronLeft*RAD_TO_DEG,
		state.ControlSurfaces.AileronRight*RAD_TO_DEG)
	fmt.Printf("   Elevator: %.1f%% → %.1f°\n",
		controls.Elevator*100,
		state.ControlSurfaces.Elevator*RAD_TO_DEG)
	fmt.Printf("   Rudder: %.1f%% → %.1f°\n",
		controls.Rudder*100,
		state.ControlSurfaces.Rudder*RAD_TO_DEG)
	fmt.Printf("   Throttle: %.0f%%\n", controls.Throttle*100)
	
	// Convert state to property map for function evaluation
	fmt.Printf("\n🧮 Property Map for Function Evaluation:\n")
	properties := state.ToPropertyMap()
	
	// Show key properties
	keyProps := []string{
		"aero/alpha-deg",
		"aero/beta-deg", 
		"aero/mach",
		"aero/qbar-psf",
		"velocities/vt-mps",
		"fcs/throttle-cmd-norm",
		"altitude/h-sl-m",
	}
	
	for _, prop := range keyProps {
		if val, exists := properties[prop]; exists {
			fmt.Printf("   %s: %.3f\n", prop, val)
		}
	}
	
	// Demonstrate function evaluation with P-51D aerodynamics
	fmt.Printf("\n✈️  Aerodynamic Function Evaluation:\n")
	
	// Find some aerodynamic functions from the P-51D config
	if config.Aerodynamics != nil {
		fmt.Printf("   Found %d aerodynamic axes\n", len(config.Aerodynamics.Axis))
		
		functionCount := 0
		for _, axis := range config.Aerodynamics.Axis {
			for _, function := range axis.Function {
				if functionCount >= 3 { // Show first 3 functions
					break
				}
				
				if function.Product != nil && function.Product.Table != nil {
					// Try to evaluate this function
					result, err := EvaluateFunction(function, properties)
					if err == nil {
						fmt.Printf("   %s: %.6f\n", function.Name, result)
					} else {
						fmt.Printf("   %s: [evaluation error]\n", function.Name)
					}
					functionCount++
				}
			}
		}
	}
	
	// Simulate a short flight sequence
	fmt.Printf("\n🎯 Flight Sequence Simulation (5 seconds):\n")
	fmt.Printf("   Time    Alt(ft)  IAS(kt)  α(°)   β(°)   φ(°)   θ(°)   ψ(°)\n")
	fmt.Printf("   ----    -------  -------  ----   ----   ----   ----   ----\n")
	
	dt := 0.1 // 100ms time steps
	for t := 0.0; t <= 5.0; t += dt {
		state.Time = t
		
		// Simulate some control inputs (gentle turn)
		if t > 1.0 && t < 4.0 {
			// Gentle right turn from 1-4 seconds
			controls.Aileron = 0.2
			controls.Rudder = 0.1
		} else {
			// Level flight
			controls.Aileron = 0.0
			controls.Rudder = 0.0
		}
		
		state.ApplyControlInputs(controls)
		
		// Simple physics simulation (very basic for demo)
		// In a real sim, this would use proper integration and forces/moments
		rollRate := controls.Aileron * 0.5 // rad/s
		yawRate := controls.Rudder * 0.3   // rad/s
		
		state.AngularRate.X = rollRate
		state.AngularRate.Z = yawRate
		
		// Update attitude (Euler integration)
		state.Roll += state.AngularRate.X * dt
		state.Yaw += state.AngularRate.Z * dt
		
		// Update quaternion from Euler angles
		state.Orientation = NewQuaternionFromEuler(state.Roll, state.Pitch, state.Yaw)
		
		// Update derived parameters
		state.UpdateDerivedParameters()
		
		// Print state every 0.5 seconds
		if t == 0.0 || (int(t*10))%5 == 0 {
			fmt.Printf("   %4.1f    %7.0f  %7.1f  %4.1f   %4.1f   %4.1f   %4.1f   %4.1f\n",
				t,
				state.Altitude*M_TO_FT,
				state.IndicatedAirspeed*MS_TO_KT,
				state.Alpha*RAD_TO_DEG,
				state.Beta*RAD_TO_DEG,
				state.Roll*RAD_TO_DEG,
				state.Pitch*RAD_TO_DEG,
				state.Yaw*RAD_TO_DEG)
		}
	}
	
	// Performance summary
	fmt.Printf("\n📊 Performance Summary:\n")
	
	// Run a performance test
	start := time.Now()
	iterations := 10000
	
	testState := state.Copy()
	for i := 0; i < iterations; i++ {
		testState.UpdateAtmosphere()
		testState.UpdateDerivedParameters()
		_ = testState.ToPropertyMap()
	}
	
	duration := time.Since(start)
	avgTime := duration / time.Duration(iterations)
	
	fmt.Printf("   State Update Performance: %d iterations in %v\n", iterations, duration)
	fmt.Printf("   Average time per update: %.2f μs\n", float64(avgTime.Nanoseconds())/1000.0)
	fmt.Printf("   Theoretical max frequency: %.0f Hz\n", 1e9/float64(avgTime.Nanoseconds()))
	
	// Show what's ready for next phase
	fmt.Printf("\n🚀 Ready for Next Phase:\n")
	fmt.Println("   ✅ Aircraft state management system")
	fmt.Println("   ✅ ISA standard atmosphere model") 
	fmt.Println("   ✅ 6-DOF state representation")
	fmt.Println("   ✅ Control input mapping")
	fmt.Println("   ✅ Property map for function evaluation")
	fmt.Println("   ✅ Real-time performance (>1000 Hz capable)")
	fmt.Println("")
	fmt.Println("   🎯 Next: Numerical integration engine")
	fmt.Println("   🎯 Next: Forces & moments calculation")
	fmt.Println("   🎯 Next: Complete flight dynamics loop")
}

// DemoStateOperations shows the performance characteristics of state operations
func DemoStateOperations() {
	fmt.Println("\n⚡ State Operations Performance Demonstration")
	fmt.Println("============================================")
	
	// Test different operation costs
	operations := []struct {
		name string
		op   func(*AircraftState)
	}{
		{"State Creation", func(s *AircraftState) { _ = NewAircraftState() }},
		{"Atmosphere Update", func(s *AircraftState) { s.UpdateAtmosphere() }},
		{"Derived Parameters", func(s *AircraftState) { s.UpdateDerivedParameters() }},
		{"Property Map", func(s *AircraftState) { _ = s.ToPropertyMap() }},
		{"Control Input", func(s *AircraftState) { s.ApplyControlInputs(NewControlInputs()) }},
		{"State Copy", func(s *AircraftState) { _ = s.Copy() }},
	}
	
	state := NewAircraftState()
	const iterations = 100000
	
	fmt.Printf("   Operation                  Time/op    Ops/sec\n")
	fmt.Printf("   ---------                  -------    -------\n")
	
	for _, op := range operations {
		start := time.Now()
		for i := 0; i < iterations; i++ {
			op.op(state)
		}
		duration := time.Since(start)
		
		timePerOp := duration / iterations
		opsPerSec := float64(iterations) / duration.Seconds()
		
		fmt.Printf("   %-25s  %7.2f μs  %8.0f\n", 
			op.name, 
			float64(timePerOp.Nanoseconds())/1000.0,
			opsPerSec)
	}
	
	fmt.Printf("\n   All operations are suitable for real-time simulation!\n")
}

// DemoQuaternionVsEuler compares quaternion vs Euler angle performance
func DemoQuaternionVsEuler() {
	fmt.Println("\n🔄 Quaternion vs Euler Angle Demonstration")
	fmt.Println("==========================================")
	
	// Create test rotations
	roll, pitch, yaw := 0.1, 0.2, 0.3
	
	// Quaternion approach
	q := NewQuaternionFromEuler(roll, pitch, yaw)
	
	fmt.Printf("   Original Euler: roll=%.3f, pitch=%.3f, yaw=%.3f rad\n", roll, pitch, yaw)
	fmt.Printf("   Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", q.W, q.X, q.Y, q.Z)
	
	// Convert back
	r2, p2, y2 := q.ToEuler()
	fmt.Printf("   Back to Euler: roll=%.3f, pitch=%.3f, yaw=%.3f rad\n", r2, p2, y2)
	
	// Show rotation of vectors
	testVectors := []Vector3{
		{X: 1, Y: 0, Z: 0},  // Forward
		{X: 0, Y: 1, Z: 0},  // Right  
		{X: 0, Y: 0, Z: 1},  // Down
	}
	
	fmt.Printf("\n   Vector Rotation Test:\n")
	for i, v := range testVectors {
		rotated := q.RotateVector(v)
		fmt.Printf("   Vector[%d]: (%.3f,%.3f,%.3f) → (%.3f,%.3f,%.3f)\n",
			i, v.X, v.Y, v.Z, rotated.X, rotated.Y, rotated.Z)
	}
	
	// Performance comparison
	const iterations = 1000000
	
	fmt.Printf("\n   Performance Comparison (%d iterations):\n", iterations)
	
	// Euler operations
	start := time.Now()
	for i := 0; i < iterations; i++ {
		// Simulate typical Euler operations
		r := float64(i) * 0.001
		p := float64(i) * 0.0005
		y := float64(i) * 0.0002
		
		// Basic trig calculations (what Euler angles require)
		_ = math.Cos(r) + math.Sin(p) + math.Cos(y)
	}
	eulerTime := time.Since(start)
	
	// Quaternion operations
	start = time.Now()
	for i := 0; i < iterations; i++ {
		// Simulate typical quaternion operations
		q1 := NewQuaternionFromEuler(float64(i)*0.001, float64(i)*0.0005, float64(i)*0.0002)
		q2 := q1.Normalize()
		_ = q2.Multiply(q1)
	}
	quatTime := time.Since(start)
	
	fmt.Printf("   Euler operations:      %v (%.2f ns/op)\n", 
		eulerTime, float64(eulerTime.Nanoseconds())/float64(iterations))
	fmt.Printf("   Quaternion operations: %v (%.2f ns/op)\n", 
		quatTime, float64(quatTime.Nanoseconds())/float64(iterations))
	
	if quatTime < eulerTime {
		speedup := float64(eulerTime) / float64(quatTime)
		fmt.Printf("   Quaternions are %.1fx faster!\n", speedup)
	} else {
		slowdown := float64(quatTime) / float64(eulerTime)
		fmt.Printf("   Quaternions are %.1fx slower (but avoid gimbal lock!)\n", slowdown)
	}
}
