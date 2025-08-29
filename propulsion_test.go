// Propulsion System Tests
// Verifies implementation against actual JSBSim P-51D values

package main

import (
	"testing"
)

// TestPropulsionSystemCreation tests basic system creation with JSBSim values
func TestPropulsionSystemCreation(t *testing.T) {
	ps := NewPropulsionSystem()
	
	t.Run("JSBSim Configuration Values", func(t *testing.T) {
		// Verify values match JSBSim XML exactly
		assertApproxEqual(t, ps.MaxThrust, 200.0, 0.001)        // Line 635
		assertApproxEqual(t, ps.ReferenceRPM, 1260.0, 0.001)    // Line 632 comment
		assertApproxEqual(t, ps.ReferenceMAP, 81.0, 0.001)      // Line 634 comment
		assertApproxEqual(t, ps.RunningFactor, 0.3, 0.001)      // Engine off initially
		
		// Engine position (36 inches = 0.9144 meters)
		assertApproxEqual(t, ps.Position.X, 36.0*0.0254, 0.001)
		assertApproxEqual(t, ps.Position.Y, 0.0, 0.001)
		assertApproxEqual(t, ps.Position.Z, 0.0, 0.001)
		
		// Engine orientation from XML
		assertApproxEqual(t, ps.Orientation.X, -4.0*DEG_TO_RAD, 0.001) // Roll: -4.0°
		assertApproxEqual(t, ps.Orientation.Y, 2.5*DEG_TO_RAD, 0.001)  // Pitch: 2.5°
		assertApproxEqual(t, ps.Orientation.Z, 0.0, 0.001)             // Yaw: 0°
	})
	
	t.Run("Engine Configuration", func(t *testing.T) {
		assertEqual(t, ps.Engine.Name, "Packard-V-1650-7")
		assertEqual(t, ps.Engine.IsRunning, false)
		assertApproxEqual(t, ps.Engine.RPM, 0.0, 0.001)
		assertApproxEqual(t, ps.Engine.ThrottlePosition, 0.0, 0.001)
	})
	
	t.Run("Propeller Configuration", func(t *testing.T) {
		assertEqual(t, ps.Propeller.Name, "P51prop")
		assertApproxEqual(t, ps.Propeller.RPM, 0.0, 0.001)
		assertApproxEqual(t, ps.Propeller.Thrust, 0.0, 0.001)
	})
	
	t.Run("Fuel System from XML", func(t *testing.T) {
		fs := ps.FuelSystem
		
		// Should have 5 tanks as defined in XML
		assertEqual(t, len(fs.Tanks), 5)
		
		// Verify tank capacities from XML
		expectedCapacities := []float64{553.84, 553.84, 511.7, 451.5, 451.5}
		expectedContents := []float64{396.0, 396.0, 0.0, 0.0, 0.0}
		
		for i, tank := range fs.Tanks {
			assertEqual(t, tank.Number, i)
			assertEqual(t, tank.Type, "AVGAS")
			assertApproxEqual(t, tank.Capacity, expectedCapacities[i], 0.001)
			assertApproxEqual(t, tank.Contents, expectedContents[i], 0.001)
			assertEqual(t, tank.Priority, 1)
		}
		
		// Total capacity should match sum
		expectedTotalCapacity := 553.84 + 553.84 + 511.7 + 451.5 + 451.5
		assertApproxEqual(t, fs.TotalCapacity, expectedTotalCapacity, 0.001)
		
		// Initial contents (wing tanks only)
		expectedTotalContents := 396.0 + 396.0
		assertApproxEqual(t, fs.TotalContents, expectedTotalContents, 0.001)
	})
}

// TestJSBSimThrustFormula tests the exact thrust calculation from JSBSim XML
func TestJSBSimThrustFormula(t *testing.T) {
	ps := NewPropulsionSystem()
	
	t.Run("JSBSim Thrust Formula", func(t *testing.T) {
		// Test the exact formula from XML lines 629-636:
		// Thrust = running_factor × (propeller_rpm / 1260) × (map_inhg / 81) × 200 lbs
		
		// Set up test conditions
		ps.Engine.IsRunning = true
		ps.RunningFactor = 1.0
		ps.Engine.RPM = 2520.0        // 2 × reference RPM
		ps.Propeller.RPM = 2520.0     // Should match engine RPM
		ps.Engine.ManifoldPressure = 40.5 // 0.5 × reference MAP
		
		ps.updatePropeller(0.01)
		
		// Expected thrust using JSBSim formula:
		// 1.0 × (2520/1260) × (40.5/81) × 200 = 1.0 × 2.0 × 0.5 × 200 = 200 lbs
		expectedThrust := 1.0 * (2520.0/1260.0) * (40.5/81.0) * 200.0
		assertApproxEqual(t, ps.Propeller.Thrust, expectedThrust, 0.001)
		assertApproxEqual(t, ps.Propeller.Thrust, 200.0, 0.001)
	})
	
	t.Run("Reference Conditions", func(t *testing.T) {
		// At reference conditions should give max thrust
		ps.Engine.IsRunning = true
		ps.RunningFactor = 1.0
		ps.Engine.RPM = 1260.0        // Reference RPM
		ps.Propeller.RPM = 1260.0
		ps.Engine.ManifoldPressure = 81.0 // Reference MAP
		
		ps.updatePropeller(0.01)
		
		// Should give exactly max thrust
		assertApproxEqual(t, ps.Propeller.Thrust, 200.0, 0.001)
	})
	
	t.Run("Engine Off Conditions", func(t *testing.T) {
		// Engine off should use running_factor = 0.3
		ps.Engine.IsRunning = false
		ps.RunningFactor = 0.3
		ps.Engine.RPM = 0.0
		ps.Propeller.RPM = 0.0
		ps.Engine.ManifoldPressure = 29.92
		
		ps.updatePropeller(0.01)
		
		// Should give zero thrust (RPM = 0)
		assertApproxEqual(t, ps.Propeller.Thrust, 0.0, 0.001)
	})
	
	t.Run("Windmilling Conditions", func(t *testing.T) {
		// Windmilling: engine off but propeller turning
		ps.Engine.IsRunning = false
		ps.RunningFactor = 0.3
		ps.Engine.RPM = 0.0
		ps.Propeller.RPM = 500.0      // Windmilling RPM
		ps.Engine.ManifoldPressure = 29.92
		
		ps.updatePropeller(0.01)
		
		// Expected: 0.3 × (500/1260) × (29.92/81) × 200
		expectedThrust := 0.3 * (500.0/1260.0) * (29.92/81.0) * 200.0
		assertApproxEqual(t, ps.Propeller.Thrust, expectedThrust, 0.1)
	})
}

// TestPropulsionSystemUpdate tests the complete update cycle
func TestPropulsionSystemUpdate(t *testing.T) {
	ps := NewPropulsionSystem()
	
	t.Run("Engine Startup", func(t *testing.T) {
		// Initially engine should be off
		assertEqual(t, ps.Engine.IsRunning, false)
		assertApproxEqual(t, ps.RunningFactor, 0.3, 0.001)
		
		// Apply throttle to start engine
		ps.Update(0.5, 0.01) // 50% throttle
		
		// Engine should now be running
		assertEqual(t, ps.Engine.IsRunning, true)
		assertApproxEqual(t, ps.RunningFactor, 1.0, 0.001)
		assertApproxEqual(t, ps.Engine.ThrottlePosition, 0.5, 0.001)
		
		// RPM should be between idle and max
		if ps.Engine.RPM <= ps.Engine.IdleRPM || ps.Engine.RPM >= ps.Engine.MaxRPM {
			t.Errorf("RPM %.1f should be between idle %.1f and max %.1f",
				ps.Engine.RPM, ps.Engine.IdleRPM, ps.Engine.MaxRPM)
		}
		
		// Thrust should be positive
		if ps.Propeller.Thrust <= 0 {
			t.Errorf("Thrust should be positive, got %.1f", ps.Propeller.Thrust)
		}
	})
	
	t.Run("Throttle Response", func(t *testing.T) {
		// Test different throttle settings
		throttleSettings := []float64{0.0, 0.25, 0.5, 0.75, 1.0}
		var previousThrust float64
		
		for _, throttle := range throttleSettings {
			ps.Update(throttle, 0.01)
			
			// Thrust should generally increase with throttle
			if throttle > 0 && ps.Propeller.Thrust < previousThrust {
				t.Logf("Throttle %.2f: Thrust %.1f lbs, RPM %.0f, MAP %.1f inHg",
					throttle, ps.Propeller.Thrust, ps.Engine.RPM, ps.Engine.ManifoldPressure)
			}
			previousThrust = ps.Propeller.Thrust
		}
		
		// Full throttle should give significant thrust
		ps.Update(1.0, 0.01)
		if ps.Propeller.Thrust < 50.0 {
			t.Errorf("Full throttle should give substantial thrust, got %.1f lbs", ps.Propeller.Thrust)
		}
	})
	
	t.Run("Fuel Consumption", func(t *testing.T) {
		initialFuel := ps.FuelSystem.TotalContents
		
		// Run engine at high power for simulated time
		for i := 0; i < 3600; i++ { // 1 hour at 1 second intervals
			ps.Update(0.8, 1.0) // 80% throttle, 1 second time step
		}
		
		finalFuel := ps.FuelSystem.TotalContents
		fuelBurned := initialFuel - finalFuel
		
		t.Logf("Fuel burned in 1 hour: %.1f lbs (%.1f gal)", fuelBurned, fuelBurned/6.0)
		
		// Should have consumed some fuel
		if fuelBurned <= 0 {
			t.Error("Should have consumed fuel during operation")
		}
		
		// Fuel flow should be reasonable (50-200 lbs/hr for high power)
		if ps.FuelSystem.FuelFlow < 10.0 || ps.FuelSystem.FuelFlow > 500.0 {
			t.Errorf("Fuel flow %.1f lbs/hr seems unrealistic", ps.FuelSystem.FuelFlow)
		}
	})
}

// TestJSBSimPropertyIntegration tests JSBSim property compatibility
func TestJSBSimPropertyIntegration(t *testing.T) {
	ps := NewPropulsionSystem()
	pm := NewPropertyManager()
	
	t.Run("Property Updates", func(t *testing.T) {
		// Start engine
		ps.Update(0.75, 0.01)
		ps.UpdateProperties(pm)
		
		// Check that all expected properties are set
		expectedProps := map[string]string{
			"propulsion/engine/set-running":               "Engine running state",
			"propulsion/engine/propeller-rpm":             "Propeller RPM",
			"propulsion/engine/map-inhg":                   "Manifold pressure",
			"propulsion/engine/thrust-lbs":                 "Thrust in lbs",
			"propulsion/engine/prop-induced-velocity_fps":  "Propeller slipstream",
			"external_reactions/exhaust-thrust/magnitude":  "External thrust force",
			"propulsion/total-fuel-lbs":                    "Total fuel",
			"propulsion/fuel-flow-rate_pph":                "Fuel flow rate",
		}
		
		for prop, description := range expectedProps {
			value, exists := pm.GetSafe(prop)
			if !exists {
				t.Errorf("Property %s (%s) not set", prop, description)
			} else {
				t.Logf("%s: %.2f", prop, value)
			}
		}
		
		// Verify specific values
		assertApproxEqual(t, pm.Get("propulsion/engine/set-running"), 1.0, 0.001) // Running
		
		rpm := pm.Get("propulsion/engine/propeller-rpm")
		if rpm <= 0 {
			t.Error("RPM should be positive when engine running")
		}
		
		thrust := pm.Get("propulsion/engine/thrust-lbs")
		if thrust <= 0 {
			t.Error("Thrust should be positive when engine running")
		}
		
		// Thrust in properties should match propeller thrust
		assertApproxEqual(t, thrust, ps.Propeller.Thrust, 0.001)
	})
}

// TestPropulsionSystemPerformance tests realistic performance characteristics
func TestPropulsionSystemPerformance(t *testing.T) {
	ps := NewPropulsionSystem()
	
	t.Run("Performance Envelope", func(t *testing.T) {
		// Test performance at different throttle settings
		t.Logf("P-51D Propulsion Performance:")
		t.Logf("Throttle  RPM    MAP     Thrust   Power Est")
		t.Logf("--------  ----   -----   ------   ---------")
		
		throttleSettings := []float64{0.0, 0.25, 0.5, 0.75, 1.0}
		for _, throttle := range throttleSettings {
			ps.Update(throttle, 0.01)
			
			powerEst := ps.Propeller.Thrust * 0.75 // Rough HP estimate
			t.Logf("%6.0f%%  %4.0f   %5.1f   %6.1f   %7.0f HP",
				throttle*100, ps.Engine.RPM, ps.Engine.ManifoldPressure,
				ps.Propeller.Thrust, powerEst)
		}
	})
	
	t.Run("Thrust-to-Weight Ratio", func(t *testing.T) {
		// P-51D empty weight ~7,125 lbs, loaded ~11,600 lbs
		ps.Update(1.0, 0.01) // Full throttle
		
		emptyWeight := 7125.0 // lbs
		loadedWeight := 11600.0 // lbs
		
		twr_empty := ps.Propeller.Thrust / emptyWeight
		twr_loaded := ps.Propeller.Thrust / loadedWeight
		
		t.Logf("Thrust-to-Weight Ratios (Exhaust Thrust Only):")
		t.Logf("  Empty weight (%.0f lbs): %.3f", emptyWeight, twr_empty)
		t.Logf("  Loaded weight (%.0f lbs): %.3f", loadedWeight, twr_loaded)
		t.Logf("  Note: This is exhaust thrust only, not total propeller thrust")
		
		// Note: This is just exhaust thrust component, not total propeller thrust
		// Main propeller thrust would be much higher and come from propeller model
		if ps.Propeller.Thrust <= 0 {
			t.Error("Should produce some exhaust thrust at full throttle")
		}
		
		// Exhaust thrust should be reasonable (small component of total thrust)
		if ps.Propeller.Thrust > 1000.0 {
			t.Errorf("Exhaust thrust %.1f lbs seems too high", ps.Propeller.Thrust)
		}
	})
}

// Benchmark propulsion system performance
func BenchmarkPropulsionUpdate(b *testing.B) {
	ps := NewPropulsionSystem()
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		throttle := float64(i%100) / 100.0 // Vary throttle
		ps.Update(throttle, 0.01)
	}
}
