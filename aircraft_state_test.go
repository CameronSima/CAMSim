package main

import (
	"math"
	"testing"
	"time"
)

// TestVector3Operations tests the Vector3 type and its operations
func TestVector3Operations(t *testing.T) {
	
	t.Run("Vector Creation and Basic Properties", func(t *testing.T) {
		v := Vector3{X: 3.0, Y: 4.0, Z: 0.0}
		
		// Test magnitude (3-4-5 triangle)
		expected := 5.0
		actual := v.Magnitude()
		assertApproxEqual(t, actual, expected, 0.001)
		
		// Test normalization
		normalized := v.Normalize()
		assertApproxEqual(t, normalized.Magnitude(), 1.0, 0.001)
		assertApproxEqual(t, normalized.X, 0.6, 0.001)
		assertApproxEqual(t, normalized.Y, 0.8, 0.001)
	})
	
	t.Run("Vector Addition and Scaling", func(t *testing.T) {
		v1 := Vector3{X: 1.0, Y: 2.0, Z: 3.0}
		v2 := Vector3{X: 4.0, Y: 5.0, Z: 6.0}
		
		// Test addition
		sum := v1.Add(v2)
		assertEqual(t, sum.X, 5.0)
		assertEqual(t, sum.Y, 7.0)
		assertEqual(t, sum.Z, 9.0)
		
		// Test scaling
		scaled := v1.Scale(2.0)
		assertEqual(t, scaled.X, 2.0)
		assertEqual(t, scaled.Y, 4.0)
		assertEqual(t, scaled.Z, 6.0)
	})
	
	t.Run("Vector Dot and Cross Products", func(t *testing.T) {
		v1 := Vector3{X: 1.0, Y: 0.0, Z: 0.0}
		v2 := Vector3{X: 0.0, Y: 1.0, Z: 0.0}
		
		// Dot product of perpendicular vectors should be zero
		dot := v1.Dot(v2)
		assertEqual(t, dot, 0.0)
		
		// Cross product should give Z direction
		cross := v1.Cross(v2)
		assertEqual(t, cross.X, 0.0)
		assertEqual(t, cross.Y, 0.0)
		assertEqual(t, cross.Z, 1.0)
	})
}

// TestQuaternionOperations tests quaternion functionality
func TestQuaternionOperations(t *testing.T) {
	
	t.Run("Euler to Quaternion Conversion", func(t *testing.T) {
		// Test identity rotation
		q := NewQuaternionFromEuler(0, 0, 0)
		assertApproxEqual(t, q.W, 1.0, 0.001)
		assertApproxEqual(t, q.X, 0.0, 0.001)
		assertApproxEqual(t, q.Y, 0.0, 0.001)
		assertApproxEqual(t, q.Z, 0.0, 0.001)
		
		// Test 90 degree roll
		q90roll := NewQuaternionFromEuler(math.Pi/2, 0, 0)
		assertApproxEqual(t, q90roll.W, math.Cos(math.Pi/4), 0.001)
		assertApproxEqual(t, q90roll.X, math.Sin(math.Pi/4), 0.001)
	})
	
	t.Run("Quaternion to Euler Conversion", func(t *testing.T) {
		originalRoll := 0.5
		originalPitch := 0.3
		originalYaw := 1.2
		
		q := NewQuaternionFromEuler(originalRoll, originalPitch, originalYaw)
		roll, pitch, yaw := q.ToEuler()
		
		assertApproxEqual(t, roll, originalRoll, 0.001)
		assertApproxEqual(t, pitch, originalPitch, 0.001)
		assertApproxEqual(t, yaw, originalYaw, 0.001)
	})
	
	t.Run("Vector Rotation", func(t *testing.T) {
		// 90 degree rotation around Z axis should turn X into Y
		q := NewQuaternionFromEuler(0, 0, math.Pi/2)
		v := Vector3{X: 1.0, Y: 0.0, Z: 0.0}
		
		rotated := q.RotateVector(v)
		assertApproxEqual(t, rotated.X, 0.0, 0.001)
		assertApproxEqual(t, rotated.Y, 1.0, 0.001)
		assertApproxEqual(t, rotated.Z, 0.0, 0.001)
	})
	
	t.Run("Quaternion Normalization", func(t *testing.T) {
		q := Quaternion{W: 2.0, X: 3.0, Y: 4.0, Z: 5.0}
		normalized := q.Normalize()
		
		magnitude := math.Sqrt(normalized.W*normalized.W + normalized.X*normalized.X + 
		                      normalized.Y*normalized.Y + normalized.Z*normalized.Z)
		assertApproxEqual(t, magnitude, 1.0, 0.001)
	})
}

// TestControlInputs tests the control input system
func TestControlInputs(t *testing.T) {
	
	t.Run("Default Control Inputs", func(t *testing.T) {
		controls := NewControlInputs()
		
		// Check safe defaults
		assertEqual(t, controls.Aileron, 0.0)
		assertEqual(t, controls.Elevator, 0.0)
		assertEqual(t, controls.Rudder, 0.0)
		assertEqual(t, controls.Throttle, 0.0)
		assertEqual(t, controls.Flaps, 0.0)
		assertEqual(t, controls.Gear, true)  // Start with gear down
		assertEqual(t, controls.Brake, 0.0)
		assertApproxEqual(t, controls.Mixture, 0.8, 0.001) // Rich mixture for start
		assertEqual(t, controls.Propeller, 1.0)
	})
	
	t.Run("Control Input Limits", func(t *testing.T) {
		controls := ControlInputs{
			Aileron:  1.5,  // Over limit
			Elevator: -1.5, // Under limit
			Throttle: 0.5,  // Normal
		}
		
		// In a real system, these would be clamped to [-1, 1] or [0, 1]
		// For now, just verify we can set them
		assertApproxEqual(t, controls.Aileron, 1.5, 0.001)
		assertApproxEqual(t, controls.Elevator, -1.5, 0.001)
		assertApproxEqual(t, controls.Throttle, 0.5, 0.001)
	})
}

// TestAircraftStateCreation tests aircraft state initialization
func TestAircraftStateCreation(t *testing.T) {
	
	t.Run("Default State Creation", func(t *testing.T) {
		state := NewAircraftState()
		
		// Check initial conditions
		assertEqual(t, state.Time, 0.0)
		assertEqual(t, state.Altitude, 1000.0) // 1000m default altitude
		assertApproxEqual(t, state.Velocity.X, 50.0, 0.001) // 50 m/s forward
		assertEqual(t, state.Velocity.Y, 0.0)
		assertEqual(t, state.Velocity.Z, 0.0)
		
		// Check that atmospheric conditions were calculated
		if state.Temperature <= 0 {
			t.Error("Temperature should be set by atmosphere calculation")
		}
		if state.Pressure <= 0 {
			t.Error("Pressure should be set by atmosphere calculation")
		}
		if state.Density <= 0 {
			t.Error("Density should be set by atmosphere calculation")
		}
	})
	
	t.Run("State Timestamp", func(t *testing.T) {
		before := time.Now()
		state := NewAircraftState()
		after := time.Now()
		
		// Timestamp should be between before and after
		if state.Timestamp.Before(before) || state.Timestamp.After(after) {
			t.Error("State timestamp should be set to current time")
		}
	})
}

// TestAtmosphereModel tests the ISA standard atmosphere implementation
func TestAtmosphereModel(t *testing.T) {
	
	t.Run("Sea Level Conditions", func(t *testing.T) {
		state := &AircraftState{Altitude: 0.0}
		state.UpdateAtmosphere()
		
		// ISA sea level conditions
		assertApproxEqual(t, state.Temperature, 288.15, 0.1)    // 15°C
		assertApproxEqual(t, state.Pressure, 101325.0, 100.0)   // 1013.25 hPa
		assertApproxEqual(t, state.Density, 1.225, 0.01)        // Standard density
		assertApproxEqual(t, state.SoundSpeed, 340.3, 1.0)      // ISA calculated ~340.3 m/s
	})
	
	t.Run("Altitude Effects", func(t *testing.T) {
		// Test at 5000m
		state := &AircraftState{Altitude: 5000.0}
		state.UpdateAtmosphere()
		
		// Temperature should decrease with altitude
		if state.Temperature >= 288.15 {
			t.Error("Temperature should decrease with altitude")
		}
		
		// Pressure should decrease with altitude
		if state.Pressure >= 101325.0 {
			t.Error("Pressure should decrease with altitude")
		}
		
		// Density should decrease with altitude
		if state.Density >= 1.225 {
			t.Error("Density should decrease with altitude")
		}
		
		// Temperature should be approximately 288.15 - 0.0065 * 5000 = 255.65K
		expected := 288.15 - 0.0065*5000.0
		assertApproxEqual(t, state.Temperature, expected, 1.0)
	})
	
	t.Run("High Altitude Conditions", func(t *testing.T) {
		// Test above 11km (stratosphere)
		state := &AircraftState{Altitude: 15000.0}
		state.UpdateAtmosphere()
		
		// Temperature should be constant at 216.65K above 11km
		assertApproxEqual(t, state.Temperature, 216.65, 1.0)
		
		// Pressure should still decrease exponentially
		if state.Pressure >= 22632.0 { // Pressure at 11km
			t.Error("Pressure should continue decreasing above 11km")
		}
	})
}

// TestDerivedParameters tests calculation of derived flight parameters
func TestDerivedParameters(t *testing.T) {
	
	t.Run("Airspeed Calculations", func(t *testing.T) {
		state := NewAircraftState()
		state.Velocity = Vector3{X: 60.0, Y: 0.0, Z: 0.0} // 60 m/s forward
		state.UpdateDerivedParameters()
		
		// True airspeed should equal velocity magnitude
		assertApproxEqual(t, state.TrueAirspeed, 60.0, 0.001)
		
		// Indicated airspeed should be slightly different due to density
		if state.IndicatedAirspeed == state.TrueAirspeed {
			t.Error("IAS should differ from TAS due to density effects")
		}
	})
	
	t.Run("Angle of Attack Calculation", func(t *testing.T) {
		state := NewAircraftState()
		
		// Level flight (no vertical velocity)
		state.Velocity = Vector3{X: 50.0, Y: 0.0, Z: 0.0}
		state.UpdateDerivedParameters()
		assertApproxEqual(t, state.Alpha, 0.0, 0.001)
		
		// Climbing (negative Z velocity in body frame)
		state.Velocity = Vector3{X: 50.0, Y: 0.0, Z: -10.0}
		state.UpdateDerivedParameters()
		if state.Alpha <= 0 {
			t.Error("Alpha should be positive when climbing")
		}
		
		// Diving (positive Z velocity in body frame)
		state.Velocity = Vector3{X: 50.0, Y: 0.0, Z: 10.0}
		state.UpdateDerivedParameters()
		if state.Alpha >= 0 {
			t.Error("Alpha should be negative when diving")
		}
	})
	
	t.Run("Sideslip Angle Calculation", func(t *testing.T) {
		state := NewAircraftState()
		
		// No sideslip
		state.Velocity = Vector3{X: 50.0, Y: 0.0, Z: 0.0}
		state.UpdateDerivedParameters()
		assertApproxEqual(t, state.Beta, 0.0, 0.001)
		
		// Right sideslip
		state.Velocity = Vector3{X: 50.0, Y: 10.0, Z: 0.0}
		state.UpdateDerivedParameters()
		if state.Beta <= 0 {
			t.Error("Beta should be positive for right sideslip")
		}
		
		// Left sideslip
		state.Velocity = Vector3{X: 50.0, Y: -10.0, Z: 0.0}
		state.UpdateDerivedParameters()
		if state.Beta >= 0 {
			t.Error("Beta should be negative for left sideslip")
		}
	})
	
	t.Run("Mach Number Calculation", func(t *testing.T) {
		state := NewAircraftState()
		state.Velocity = Vector3{X: 343.0, Y: 0.0, Z: 0.0} // Speed of sound at sea level
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		// Should be approximately Mach 1
		assertApproxEqual(t, state.Mach, 1.0, 0.1)
	})
}

// TestControlSurfaceMapping tests the mapping from control inputs to surface positions
func TestControlSurfaceMapping(t *testing.T) {
	
	t.Run("Control Input to Surface Position", func(t *testing.T) {
		state := NewAircraftState()
		controls := ControlInputs{
			Aileron:  0.5,  // Half right
			Elevator: -0.3, // Slight down
			Rudder:   0.2,  // Slight right
			Flaps:    0.25, // Quarter flaps
		}
		
		state.ApplyControlInputs(controls)
		
		// Check aileron mapping (opposite for left/right)
		if state.ControlSurfaces.AileronLeft <= 0 {
			t.Error("Left aileron should be positive (down) for right roll command")
		}
		if state.ControlSurfaces.AileronRight >= 0 {
			t.Error("Right aileron should be negative (up) for right roll command")
		}
		
		// Check elevator mapping
		if state.ControlSurfaces.Elevator >= 0 {
			t.Error("Elevator should be negative for down command")
		}
		
		// Check rudder mapping
		if state.ControlSurfaces.Rudder <= 0 {
			t.Error("Rudder should be positive for right command")
		}
		
		// Check flap mapping
		expectedFlap := 0.25 * 40.0 * DEG_TO_RAD // 25% of 40° max
		assertApproxEqual(t, state.ControlSurfaces.FlapLeft, expectedFlap, 0.001)
	})
	
	t.Run("Gear Control", func(t *testing.T) {
		state := NewAircraftState()
		
		// Gear up
		controls := ControlInputs{Gear: false}
		state.ApplyControlInputs(controls)
		assertEqual(t, state.Gear.Down, false)
		assertEqual(t, state.Gear.Transition, 0.0)
		
		// Gear down
		controls.Gear = true
		state.ApplyControlInputs(controls)
		assertEqual(t, state.Gear.Down, true)
		assertEqual(t, state.Gear.Transition, 1.0)
	})
}

// TestPropertyMap tests the conversion to property map for function evaluation
func TestPropertyMap(t *testing.T) {
	
	t.Run("Property Map Generation", func(t *testing.T) {
		state := NewAircraftState()
		state.Alpha = 0.1 // 0.1 radians
		state.Beta = 0.05
		state.Mach = 0.3
		
		props := state.ToPropertyMap()
		
		// Check key properties are present
		_, hasAlphaRad := props["aero/alpha-rad"]
		_, hasAlphaDeg := props["aero/alpha-deg"]
		_, hasMach := props["aero/mach"]
		_, hasQbar := props["aero/qbar-Pa"]
		
		if !hasAlphaRad || !hasAlphaDeg || !hasMach || !hasQbar {
			t.Error("Property map should contain key aerodynamic properties")
		}
		
		// Check unit conversions
		assertApproxEqual(t, props["aero/alpha-rad"], 0.1, 0.001)
		assertApproxEqual(t, props["aero/alpha-deg"], 0.1*RAD_TO_DEG, 0.001)
		assertApproxEqual(t, props["aero/mach"], 0.3, 0.001)
	})
	
	t.Run("Control Property Mapping", func(t *testing.T) {
		state := NewAircraftState()
		controls := ControlInputs{
			Aileron:  0.5,
			Elevator: -0.3,
			Throttle: 0.8,
			Gear:     true,
		}
		state.ApplyControlInputs(controls)
		
		props := state.ToPropertyMap()
		
		assertApproxEqual(t, props["fcs/aileron-cmd-norm"], 0.5, 0.001)
		assertApproxEqual(t, props["fcs/elevator-cmd-norm"], -0.3, 0.001)
		assertApproxEqual(t, props["fcs/throttle-cmd-norm"], 0.8, 0.001)
		assertEqual(t, props["fcs/gear-cmd-norm"], 1.0) // true -> 1.0
	})
}

// TestStateCopy tests state copying functionality
func TestStateCopy(t *testing.T) {
	
	t.Run("Deep Copy Verification", func(t *testing.T) {
		original := NewAircraftState()
		original.Time = 123.45
		original.Altitude = 2000.0
		original.Alpha = 0.15
		
		copy := original.Copy()
		
		// Verify values are copied
		assertEqual(t, copy.Time, 123.45)
		assertEqual(t, copy.Altitude, 2000.0)
		assertApproxEqual(t, copy.Alpha, 0.15, 0.001)
		
		// Verify independence (changing copy doesn't affect original)
		copy.Time = 999.0
		copy.Altitude = 5000.0
		
		assertEqual(t, original.Time, 123.45)   // Should be unchanged
		assertEqual(t, original.Altitude, 2000.0) // Should be unchanged
	})
}

// TestStateStringRepresentation tests the string output
func TestStateStringRepresentation(t *testing.T) {
	
	t.Run("String Format", func(t *testing.T) {
		state := NewAircraftState()
		state.Time = 60.0
		state.Altitude = 1500.0
		state.IndicatedAirspeed = 50.0 // m/s
		state.Alpha = 0.1 // radians
		
		str := state.String()
		
		// Should contain key information
		if len(str) == 0 {
			t.Error("String representation should not be empty")
		}
		
		// Basic format check (should contain time, altitude, etc.)
		// This is a simple check - in practice you might want more specific validation
		t.Logf("State string: %s", str)
	})
}

// BenchmarkStateOperations benchmarks key state operations
func BenchmarkStateOperations(b *testing.B) {
	
	b.Run("NewAircraftState", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			_ = NewAircraftState()
		}
	})
	
	b.Run("UpdateAtmosphere", func(b *testing.B) {
		state := NewAircraftState()
		for i := 0; i < b.N; i++ {
			state.Altitude = float64(i % 10000) // Vary altitude
			state.UpdateAtmosphere()
		}
	})
	
	b.Run("UpdateDerivedParameters", func(b *testing.B) {
		state := NewAircraftState()
		for i := 0; i < b.N; i++ {
			state.Velocity.X = 50.0 + float64(i%20) // Vary velocity
			state.UpdateDerivedParameters()
		}
	})
	
	b.Run("ToPropertyMap", func(b *testing.B) {
		state := NewAircraftState()
		for i := 0; i < b.N; i++ {
			_ = state.ToPropertyMap()
		}
	})
	
	b.Run("ApplyControlInputs", func(b *testing.B) {
		state := NewAircraftState()
		controls := NewControlInputs()
		for i := 0; i < b.N; i++ {
			controls.Aileron = float64(i%100) / 100.0 // Vary input
			state.ApplyControlInputs(controls)
		}
	})
}
