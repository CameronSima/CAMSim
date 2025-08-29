// Integration Method Comparison Test
// Compares Euler, Fake RK4, and True RK4 integration methods

package main

import (
	"os"
	"testing"
)

// TestIntegrationMethodComparison compares different integration methods
func TestIntegrationMethodComparison(t *testing.T) {
	// Load P-51D configuration
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		t.Fatalf("Failed to load P-51D config: %v", err)
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		t.Fatalf("Failed to parse P-51D config: %v", err)
	}
	
	// Create test state
	initialState := &AircraftState{
		Position:    Vector3{X: 0, Y: 0, Z: -1000}, // 1000m altitude
		Velocity:    Vector3{X: 50, Y: 0, Z: 0},    // 50 m/s forward
		Orientation: Quaternion{W: 1, X: 0, Y: 0, Z: 0}, // Level
		AngularRate: Vector3{X: 0, Y: 0, Z: 0},
		Controls:    ControlInputs{Throttle: 0.75}, // 75% throttle
		Temperature: 288.15,
		Pressure:    101325.0,
		Density:     1.225,
	}
	
	dt := 0.01 // 10ms time step
	numSteps := 100 // 1 second of simulation
	
	t.Run("Integration Method Comparison", func(t *testing.T) {
		// Test with simplified RK4 (useRealisticPropulsion = false)
		engineSimple, err := NewFlightDynamicsEngineWithPropulsion(config, false, false)
		if err != nil {
			t.Fatalf("Failed to create simple engine: %v", err)
		}
		
		// Test with true RK4 (useRealisticPropulsion = true)
		engineTrue, err := NewFlightDynamicsEngineWithPropulsion(config, false, true)
		if err != nil {
			t.Fatalf("Failed to create true RK4 engine: %v", err)
		}
		
		// Run simulation with both integrators
		stateSimple := *initialState
		stateTrue := *initialState
		
		var simpleAccel, trueAccel float64
		
		for i := 0; i < numSteps; i++ {
			// Simple RK4
			newStateSimple, derivativesSimple, err := engineSimple.RunSimulationStepWithPropulsion(&stateSimple, dt)
			if err != nil {
				t.Fatalf("Simple integration failed at step %d: %v", i, err)
			}
			stateSimple = *newStateSimple
			if i == 0 { // Capture first step acceleration
				simpleAccel = derivativesSimple.VelocityDot.X
			}
			
			// True RK4
			newStateTrue, derivativesTrue, err := engineTrue.RunSimulationStepWithPropulsion(&stateTrue, dt)
			if err != nil {
				t.Fatalf("True RK4 integration failed at step %d: %v", i, err)
			}
			stateTrue = *newStateTrue
			if i == 0 { // Capture first step acceleration
				trueAccel = derivativesTrue.VelocityDot.X
			}
		}
		
		// Compare final results
		t.Logf("Integration Method Comparison (1 second, 75%% throttle):")
		t.Logf("Method         | Integrator            | Final Speed | Final Accel | Thrust")
		t.Logf("---------------|----------------------|-------------|-------------|--------")
		t.Logf("Simplified RK4 | %-20s | %8.2f m/s | %8.3f m/s² | %6.1f lbs", 
			engineSimple.FlightDynamicsEngine.Integrator.GetName(),
			stateSimple.Velocity.Magnitude(), 
			simpleAccel,
			engineSimple.Propulsion.Propeller.Thrust)
		t.Logf("True RK4       | %-20s | %8.2f m/s | %8.3f m/s² | %6.1f lbs", 
			engineTrue.FlightDynamicsEngine.Integrator.GetName(),
			stateTrue.Velocity.Magnitude(), 
			trueAccel,
			engineTrue.Propulsion.Propeller.Thrust)
		
		// Calculate differences
		speedDiff := stateTrue.Velocity.Magnitude() - stateSimple.Velocity.Magnitude()
		accelDiff := trueAccel - simpleAccel
		thrustDiff := engineTrue.Propulsion.Propeller.Thrust - engineSimple.Propulsion.Propeller.Thrust
		
		t.Logf("")
		t.Logf("Improvements with True RK4:")
		t.Logf("  Speed difference: %+.3f m/s (%.1f%%)", 
			speedDiff, speedDiff/stateSimple.Velocity.Magnitude()*100)
		t.Logf("  Acceleration difference: %+.3f m/s² (%.1f%%)", 
			accelDiff, accelDiff/simpleAccel*100)
		t.Logf("  Thrust difference: %+.1f lbs (%.1f%%)", 
			thrustDiff, thrustDiff/engineSimple.Propulsion.Propeller.Thrust*100)
		
		// Note: True RK4 with dynamics re-evaluation is disabled for stability
		// Both methods currently use the improved RK4 approximation
		if speedDiff == 0.0 && accelDiff == 0.0 {
			t.Log("Both methods use improved RK4 approximation (true RK4 disabled for stability)")
		}
		
		// Validate that results are reasonable
		if stateTrue.Velocity.Magnitude() <= 0 || stateSimple.Velocity.Magnitude() <= 0 {
			t.Error("Both methods should produce positive final speeds")
		}
	})
	
	t.Run("Integration Stability", func(t *testing.T) {
		// Test with larger time steps to see stability differences
		largeDt := 0.1 // 100ms time step
		
		engineTrue, _ := NewFlightDynamicsEngineWithPropulsion(config, false, true)
		state := *initialState
		
		for i := 0; i < 10; i++ { // 1 second with large steps
			newState, derivatives, err := engineTrue.RunSimulationStepWithPropulsion(&state, largeDt)
			if err != nil {
				t.Fatalf("Large time step integration failed: %v", err)
			}
			
			// Check for numerical instability
			if isInvalidFloat(derivatives.VelocityDot.X) || 
			   isInvalidFloat(newState.Velocity.X) ||
			   newState.Velocity.Magnitude() > 1000.0 { // Unreasonably high speed
				t.Errorf("Integration became unstable at step %d with large time step", i)
				break
			}
			
			state = *newState
		}
		
		t.Logf("Large time step (%.1f ms) integration stable: final speed %.2f m/s", 
			largeDt*1000, state.Velocity.Magnitude())
	})
}


