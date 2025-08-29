// Simple Propulsion Integration Test

package main

import (
	"os"
	"testing"
)

// TestBasicPropulsionIntegration tests basic propulsion integration
func TestBasicPropulsionIntegration(t *testing.T) {
	// Use the actual P-51D config for proper testing
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		t.Fatalf("Failed to load P-51D config: %v", err)
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		t.Fatalf("Failed to parse P-51D config: %v", err)
	}
	
	engine, err := NewFlightDynamicsEngineWithPropulsion(config, false, true)
	if err != nil {
		t.Fatalf("Failed to create integrated engine: %v", err)
	}
	
	// Test basic functionality
	t.Run("Engine Creation", func(t *testing.T) {
		if engine.Propulsion == nil {
			t.Error("Propulsion system not created")
		}
		
		if engine.FCS == nil {
			t.Error("Flight control system not created")
		}
		
		// Initial state checks
		assertEqual(t, engine.Propulsion.Engine.IsRunning, false)
		if engine.Propulsion.FuelSystem.TotalContents <= 0 {
			t.Error("No initial fuel")
		}
		
		t.Logf("Initial fuel: %.1f lbs", engine.Propulsion.FuelSystem.TotalContents)
	})
	
	t.Run("Throttle Response", func(t *testing.T) {
		// Create simple state
		state := &AircraftState{
			Position:    Vector3{X: 0, Y: 0, Z: -1000},
			Velocity:    Vector3{X: 50, Y: 0, Z: 0},
			Orientation: Quaternion{W: 1, X: 0, Y: 0, Z: 0},
			AngularRate: Vector3{X: 0, Y: 0, Z: 0},
			Controls:    ControlInputs{Throttle: 0.75},
			Temperature: 288.15,
			Pressure:    101325.0,
			Density:     1.225,
		}
		
		// Run simulation step
		newState, derivatives, err := engine.RunSimulationStepWithPropulsion(state, 0.01)
		if err != nil {
			t.Fatalf("Simulation step failed: %v", err)
		}
		
		// Basic checks
		if !engine.Propulsion.Engine.IsRunning {
			t.Error("Engine should be running with 75% throttle")
		}
		
		if engine.Propulsion.Propeller.Thrust <= 0 {
			t.Error("Should have positive thrust")
		}
		
		if derivatives.VelocityDot.X <= 0 {
			t.Logf("DEBUG: VelocityDot.X = %.6f", derivatives.VelocityDot.X)
			t.Logf("DEBUG: VelocityDot.Y = %.6f", derivatives.VelocityDot.Y)
			t.Logf("DEBUG: VelocityDot.Z = %.6f", derivatives.VelocityDot.Z)
			t.Error("Should have forward acceleration from thrust")
		}
		
		t.Logf("Engine RPM: %.0f", engine.Propulsion.Engine.RPM)
		t.Logf("Thrust: %.1f lbs", engine.Propulsion.Propeller.Thrust)
		t.Logf("Forward acceleration: %.3f m/s²", derivatives.VelocityDot.X)
		
		// Test that properties now have real values (not placeholders)
		alpha := engine.FCS.Properties.Get("velocities/alpha-rad")
		roll := engine.FCS.Properties.Get("attitude/phi-rad")
		t.Logf("Angle of attack: %.3f rad (%.1f°)", alpha, alpha*180/3.14159)
		t.Logf("Roll angle: %.3f rad (%.1f°)", roll, roll*180/3.14159)
		
		// Validate no NaN/Inf values
		if isInvalidFloat(derivatives.VelocityDot.X) {
			t.Error("Invalid acceleration value")
		}
		if isInvalidFloat(newState.Velocity.X) {
			t.Error("Invalid velocity value")
		}
	})
}

// Helper function to check for invalid float values
func isInvalidFloat(f float64) bool {
	return f != f || f > 1e10 || f < -1e10 // NaN or extreme values
}
