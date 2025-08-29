// Flight Control System Tests
// Comprehensive tests for the FCS implementation

package main

import (
	"math"
	"os"
	"testing"
	"time"
)

// =============================================================================
// PROPERTY MANAGER TESTS
// =============================================================================

func TestPropertyManager(t *testing.T) {
	pm := NewPropertyManager()
	
	t.Run("Basic Set/Get", func(t *testing.T) {
		pm.Set("test/property", 42.5)
		value := pm.Get("test/property")
		assertApproxEqual(t, value, 42.5, 0.001)
	})
	
	t.Run("Non-existent Property", func(t *testing.T) {
		value := pm.Get("non/existent")
		assertApproxEqual(t, value, 0.0, 0.001)
	})
	
	t.Run("Property Aliases", func(t *testing.T) {
		pm.Set("original/property", 123.0)
		pm.SetAlias("alias/property", "original/property")
		
		value := pm.Get("alias/property")
		assertApproxEqual(t, value, 123.0, 0.001)
		
		// Setting via alias should update original
		pm.Set("alias/property", 456.0)
		original := pm.Get("original/property")
		assertApproxEqual(t, original, 456.0, 0.001)
	})
	
	t.Run("Standard Properties Initialization", func(t *testing.T) {
		// Check that standard JSBSim properties are initialized
		fcsProps := []string{
			"fcs/aileron-cmd-norm",
			"fcs/elevator-cmd-norm", 
			"fcs/rudder-cmd-norm",
			"fcs/throttle-cmd-norm",
		}
		
		for _, prop := range fcsProps {
			value, exists := pm.GetSafe(prop)
			if !exists {
				t.Errorf("Standard property %s not initialized", prop)
			}
			assertApproxEqual(t, value, 0.0, 0.001)
		}
	})
	
	t.Run("Aircraft State Synchronization", func(t *testing.T) {
		state := NewAircraftState()
		state.Controls.Elevator = 0.5
		state.Controls.Aileron = -0.3
		state.Altitude = 1500.0
		state.Velocity = Vector3{X: 80.0, Y: 5.0, Z: -2.0}
		
		pm.UpdateFromAircraftState(state)
		
		assertApproxEqual(t, pm.Get("fcs/elevator-cmd-norm"), 0.5, 0.001)
		assertApproxEqual(t, pm.Get("fcs/aileron-cmd-norm"), -0.3, 0.001)
		assertApproxEqual(t, pm.Get("position/h-sl-ft"), 1500.0*3.28084, 0.1)
	})
}

// =============================================================================
// COMPONENT TESTS
// =============================================================================

func TestActuatorComponent(t *testing.T) {
	pm := NewPropertyManager()
	
	t.Run("Basic Actuator Operation", func(t *testing.T) {
		actuator := NewActuatorComponent("test-actuator", "input", "output")
		
		pm.Set("input", 0.5)
		output := actuator.Execute(pm, 0.01)
		
		assertApproxEqual(t, output, 0.5, 0.001)
		assertApproxEqual(t, pm.Get("output"), 0.5, 0.001)
	})
	
	t.Run("Rate Limiting", func(t *testing.T) {
		actuator := NewActuatorComponent("rate-limited", "input", "output")
		actuator.SetRateLimit(10.0) // 10 units/sec max rate
		
		// Step input from 0 to 1
		pm.Set("input", 1.0)
		dt := 0.05 // 50ms time step
		
		// First step should be rate limited
		output := actuator.Execute(pm, dt)
		expectedChange := 10.0 * dt // 0.5 units max change
		assertApproxEqual(t, output, expectedChange, 0.001)
		
		// Continue until settled
		for i := 0; i < 5; i++ {
			output = actuator.Execute(pm, dt)
		}
		
		// Should eventually reach commanded value
		assertApproxEqual(t, output, 1.0, 0.01)
	})
	
	t.Run("Lag Filtering", func(t *testing.T) {
		actuator := NewActuatorComponent("lag-test", "input", "output")
		actuator.SetLag(0.1) // 100ms time constant
		
		pm.Set("input", 1.0)
		dt := 0.01
		
		// Should not reach commanded value immediately
		output := actuator.Execute(pm, dt)
		if output >= 0.9 {
			t.Errorf("Lag filter not working: output %.3f too close to input", output)
		}
		
		// Execute many steps to approach steady state
		for i := 0; i < 100; i++ {
			output = actuator.Execute(pm, dt)
		}
		
		// Should approach commanded value
		assertApproxEqual(t, output, 1.0, 0.05)
	})
	
	t.Run("Hysteresis", func(t *testing.T) {
		actuator := NewActuatorComponent("hysteresis-test", "input", "output")
		actuator.SetHysteresis(0.1) // 10% hysteresis band
		
		dt := 0.01
		
		// Set initial value
		pm.Set("input", 0.5)
		actuator.Execute(pm, dt)
		
		// Small change within hysteresis band should be ignored
		pm.Set("input", 0.52)
		output := actuator.Execute(pm, dt)
		assertApproxEqual(t, output, 0.5, 0.001) // Should stay at previous value
		
		// Large change outside hysteresis band should be processed
		pm.Set("input", 0.8)
		output = actuator.Execute(pm, dt)
		assertApproxEqual(t, output, 0.8, 0.001)
	})
}

func TestLagFilterComponent(t *testing.T) {
	pm := NewPropertyManager()
	
	t.Run("Basic Lag Filter", func(t *testing.T) {
		filter := NewLagFilterComponent("test-lag", "input", "output", 0.1)
		
		pm.Set("input", 1.0)
		dt := 0.01
		
		// First execution should not reach full value
		output := filter.Execute(pm, dt)
		if output >= 0.9 {
			t.Errorf("Lag filter response too fast: %.3f", output)
		}
		
		// Execute multiple steps
		for i := 0; i < 50; i++ {
			output = filter.Execute(pm, dt)
		}
		
		// Should approach input value
		assertApproxEqual(t, output, 1.0, 0.05)
	})
	
	t.Run("Zero Time Constant", func(t *testing.T) {
		filter := NewLagFilterComponent("no-lag", "input", "output", 0.0)
		
		pm.Set("input", 0.75)
		output := filter.Execute(pm, 0.01)
		
		// Should pass through directly
		assertApproxEqual(t, output, 0.75, 0.001)
	})
}

func TestGainComponent(t *testing.T) {
	pm := NewPropertyManager()
	
	t.Run("Positive Gain", func(t *testing.T) {
		gain := NewGainComponent("test-gain", "input", "output", 2.5)
		
		pm.Set("input", 0.4)
		output := gain.Execute(pm, 0.01)
		
		assertApproxEqual(t, output, 1.0, 0.001)
		assertApproxEqual(t, pm.Get("output"), 1.0, 0.001)
	})
	
	t.Run("Negative Gain", func(t *testing.T) {
		gain := NewGainComponent("invert-gain", "input", "output", -1.0)
		
		pm.Set("input", 0.5)
		output := gain.Execute(pm, 0.01)
		
		assertApproxEqual(t, output, -0.5, 0.001)
	})
}

func TestSummerComponent(t *testing.T) {
	pm := NewPropertyManager()
	
	t.Run("Basic Addition", func(t *testing.T) {
		summer := NewSummerComponent("test-sum", []string{"input1", "input2"}, "output")
		
		pm.Set("input1", 0.3)
		pm.Set("input2", 0.7)
		output := summer.Execute(pm, 0.01)
		
		assertApproxEqual(t, output, 1.0, 0.001)
	})
	
	t.Run("Addition and Subtraction", func(t *testing.T) {
		summer := NewSummerComponent("add-sub", []string{"input1", "input2"}, "output")
		summer.SetSigns([]float64{1.0, -1.0}) // Add first, subtract second
		
		pm.Set("input1", 1.5)
		pm.Set("input2", 0.5)
		output := summer.Execute(pm, 0.01)
		
		assertApproxEqual(t, output, 1.0, 0.001)
	})
	
	t.Run("With Bias", func(t *testing.T) {
		summer := NewSummerComponent("biased-sum", []string{"input1"}, "output")
		summer.SetBias(0.2)
		
		pm.Set("input1", 0.3)
		output := summer.Execute(pm, 0.01)
		
		assertApproxEqual(t, output, 0.5, 0.001)
	})
}

func TestSwitchComponent(t *testing.T) {
	pm := NewPropertyManager()
	
	t.Run("Basic Switch GT", func(t *testing.T) {
		sw := NewSwitchComponent("test-switch", "output")
		sw.SetTest("test-prop", "GT", 0.5)
		sw.SetValues(1.0, 0.0) // true=1.0, false=0.0
		
		// Test condition false
		pm.Set("test-prop", 0.3)
		output := sw.Execute(pm, 0.01)
		assertApproxEqual(t, output, 0.0, 0.001)
		
		// Test condition true
		pm.Set("test-prop", 0.8)
		output = sw.Execute(pm, 0.01)
		assertApproxEqual(t, output, 1.0, 0.001)
	})
	
	t.Run("Switch with Input Properties", func(t *testing.T) {
		sw := NewSwitchComponent("input-switch", "output")
		sw.SetTest("condition", "GE", 1.0)
		sw.SetInputs("true-input", "false-input")
		
		pm.Set("true-input", 0.8)
		pm.Set("false-input", 0.2)
		
		// Condition false
		pm.Set("condition", 0.5)
		output := sw.Execute(pm, 0.01)
		assertApproxEqual(t, output, 0.2, 0.001)
		
		// Condition true
		pm.Set("condition", 1.5)
		output = sw.Execute(pm, 0.01)
		assertApproxEqual(t, output, 0.8, 0.001)
	})
}

// =============================================================================
// RATE GROUP TESTS
// =============================================================================

func TestRateGroupScheduler(t *testing.T) {
	pm := NewPropertyManager()
	
	t.Run("Rate Group Creation", func(t *testing.T) {
		rg := NewRateGroupScheduler("test-group", 60.0)
		
		assertEqual(t, rg.Name, "test-group")
		assertApproxEqual(t, rg.RateHz, 60.0, 0.001)
		assertApproxEqual(t, rg.Period, 1.0/60.0, 0.001)
		assertEqual(t, len(rg.Components), 0)
	})
	
	t.Run("Component Assignment", func(t *testing.T) {
		rg := NewRateGroupScheduler("comp-group", 30.0)
		comp := NewGainComponent("test-comp", "input", "output", 1.0)
		
		rg.AddComponent(comp)
		
		assertEqual(t, len(rg.Components), 1)
		assertEqual(t, comp.GetRateGroup(), "comp-group")
	})
	
	t.Run("Execution Scheduling", func(t *testing.T) {
		rg := NewRateGroupScheduler("sched-test", 100.0) // 100 Hz = 10ms period
		comp := NewGainComponent("sched-comp", "input", "output", 2.0)
		rg.AddComponent(comp)
		
		pm.Set("input", 0.5)
		
		// Should execute immediately on first call
		now := time.Now()
		if !rg.ShouldExecute(now) {
			t.Error("Rate group should execute on first call")
		}
		
		rg.Execute(pm, 0.01)
		assertApproxEqual(t, pm.Get("output"), 1.0, 0.001)
		
		// Should not execute immediately again
		if rg.ShouldExecute(time.Now()) {
			t.Error("Rate group should not execute immediately after execution")
		}
		
		// Should execute after sufficient time
		time.Sleep(15 * time.Millisecond) // Wait longer than period
		if !rg.ShouldExecute(time.Now()) {
			t.Error("Rate group should execute after period elapsed")
		}
	})
}

// =============================================================================
// FLIGHT CONTROL SYSTEM TESTS
// =============================================================================

func TestFlightControlSystem(t *testing.T) {
	t.Run("FCS Creation", func(t *testing.T) {
		fcs := NewFlightControlSystem("Test FCS", 120.0)
		
		assertEqual(t, fcs.Name, "Test FCS")
		assertApproxEqual(t, fcs.DefaultRate, 120.0, 0.001)
		assertEqual(t, len(fcs.RateGroups), 1) // Default group
		assertEqual(t, len(fcs.Components), 0)
	})
	
	t.Run("Component Registration", func(t *testing.T) {
		fcs := NewFlightControlSystem("Component Test", 60.0)
		comp := NewGainComponent("test-comp", "input", "output", 1.5)
		
		fcs.AddComponent(comp)
		
		assertEqual(t, len(fcs.Components), 1)
		assertEqual(t, fcs.GetComponent("test-comp"), comp)
		
		// Should be assigned to default rate group
		defaultGroup := fcs.GetRateGroup("default")
		assertEqual(t, len(defaultGroup.Components), 1)
	})
	
	t.Run("Custom Rate Groups", func(t *testing.T) {
		fcs := NewFlightControlSystem("Rate Test", 60.0)
		fcs.AddRateGroup("fast", 200.0)
		fcs.AddRateGroup("slow", 10.0)
		
		assertEqual(t, len(fcs.RateGroups), 3) // default + 2 custom
		
		fastGroup := fcs.GetRateGroup("fast")
		assertApproxEqual(t, fastGroup.RateHz, 200.0, 0.001)
		
		slowGroup := fcs.GetRateGroup("slow")
		assertApproxEqual(t, slowGroup.RateHz, 10.0, 0.001)
	})
	
	t.Run("Full System Execution", func(t *testing.T) {
		fcs := NewFlightControlSystem("Exec Test", 100.0)
		
		// Create a simple processing chain: input → gain → actuator → output
		gain := NewGainComponent("gain", "fcs/elevator-cmd-norm", "gain-output", 25.0)
		actuator := NewActuatorComponent("actuator", "gain-output", "fcs/elevator-pos-rad")
		actuator.SetLag(0.05) // 50ms lag
		
		fcs.AddComponent(gain)
		fcs.AddComponent(actuator)
		
		// Create test state
		state := NewAircraftState()
		state.Controls.Elevator = 0.4 // 40% elevator input
		
		// Execute system
		fcs.Execute(state, 0.01)
		
		// Debug: check input property
		elevatorInput := fcs.Properties.Get("fcs/elevator-cmd-norm")
		if elevatorInput != 0.4 {
			t.Errorf("Elevator input not set correctly: expected 0.4, got %.3f", elevatorInput)
		}
		
		// Check intermediate values
		gainOutput := fcs.Properties.Get("gain-output")
		assertApproxEqual(t, gainOutput, 10.0, 0.001) // 0.4 * 25.0
		
		// Actuator output should be less due to lag (50ms lag with 10ms step)
		actuatorOutput := fcs.Properties.Get("fcs/elevator-pos-rad")
		t.Logf("First step: gain output = %.3f, actuator output = %.3f", gainOutput, actuatorOutput)
		
		// First step should be significantly less than target due to lag
		expectedFirst := 10.0 * 0.01 / (0.05 + 0.01) // ~1.67
		assertApproxEqual(t, actuatorOutput, expectedFirst, 0.1)
		
		// Execute multiple steps to approach steady state
		// With 50ms lag and 10ms steps, it takes about 5 time constants (250ms = 25 steps) to reach 99%
		for i := 0; i < 50; i++ {
			fcs.Execute(state, 0.01)
		}
		
		actuatorOutput = fcs.Properties.Get("fcs/elevator-pos-rad")
		assertApproxEqual(t, actuatorOutput, 10.0, 1.0) // Should approach gain output
	})
}

// =============================================================================
// INTEGRATION TESTS
// =============================================================================

func TestFlightDynamicsEngineWithFCS(t *testing.T) {
	// Load P-51D configuration
	file, err := os.Open("/Users/cameronsima/dev/camSIM_go/aircraft/p51d-jsbsim.xml")
	if err != nil {
		t.Skipf("Skipping FCS integration test: %v", err)
	}
	defer file.Close()
	
	config, err := ParseJSBSimConfig(file)
	if err != nil {
		t.Skipf("Skipping FCS integration test: %v", err)
	}
	
	t.Run("Engine Creation", func(t *testing.T) {
		engine, err := NewFlightDynamicsEngineWithFCS(config, true)
		if err != nil {
			t.Fatalf("Failed to create FCS engine: %v", err)
		}
		
		if engine.FCS == nil {
			t.Error("FCS not initialized")
		}
		
		if len(engine.FCS.Components) == 0 {
			t.Error("No FCS components loaded")
		}
		
		assertEqual(t, engine.UseRealisticControls, true)
	})
	
	t.Run("Control Input Processing", func(t *testing.T) {
		engine, err := NewFlightDynamicsEngineWithFCS(config, true)
		if err != nil {
			t.Fatalf("Failed to create engine: %v", err)
		}
		
		controls := ControlInputs{
			Elevator: 0.3,
			Aileron:  -0.2,
			Rudder:   0.1,
			Throttle: 0.8,
		}
		
		engine.SetControlInputs(controls)
		
		// Check that pilot commands are set in properties
		assertApproxEqual(t, engine.FCS.Properties.Get("fcs/elevator-cmd-norm"), 0.3, 0.001)
		assertApproxEqual(t, engine.FCS.Properties.Get("fcs/aileron-cmd-norm"), -0.2, 0.001)
		assertApproxEqual(t, engine.FCS.Properties.Get("fcs/rudder-cmd-norm"), 0.1, 0.001)
		assertApproxEqual(t, engine.FCS.Properties.Get("fcs/throttle-cmd-norm"), 0.8, 0.001)
	})
	
	t.Run("Simulation Step with FCS", func(t *testing.T) {
		engine, err := NewFlightDynamicsEngineWithFCS(config, true)
		if err != nil {
			t.Fatalf("Failed to create engine: %v", err)
		}
		
		// Create initial state
		state := NewAircraftState()
		state.Altitude = 3000.0
		state.Position.Z = -3000.0
		state.Velocity = Vector3{X: 100.0, Y: 0.0, Z: 0.0}
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
		
		// Set control inputs on the state (so they persist through FCS.Execute)
		controls := ControlInputs{Elevator: 0.2, Throttle: 0.7}
		engine.SetControlInputsOnState(state, controls)
		
		// Run multiple simulation steps to allow actuator to respond
		var newState *AircraftState
		var derivatives *StateDerivatives
		
		for i := 0; i < 10; i++ {
			newState, derivatives, err = engine.RunSimulationStepWithFCS(state, 1.0/60.0)
			if err != nil {
				t.Fatalf("Simulation step failed: %v", err)
			}
			state = newState
		}
		
		if newState == nil {
			t.Error("New state is nil")
		}
		
		if derivatives == nil {
			t.Error("Derivatives is nil")
		}
		
		// Check that control surfaces have been processed by FCS
		surfaces := engine.GetControlSurfacePositions()
		if len(surfaces) == 0 {
			t.Error("No control surface positions returned")
		}
		
		// Elevator should be non-zero due to input (after multiple steps)
		if math.Abs(surfaces["elevator"]) < 0.01 {
			t.Error("Elevator position not processed by FCS")
		}
	})
}

// =============================================================================
// PERFORMANCE TESTS
// =============================================================================

func TestFCSPerformance(t *testing.T) {
	t.Run("Basic FCS Performance", func(t *testing.T) {
		fcs := CreateBasicFlightControlSystem()
		state := NewAircraftState()
		
		// Warm up
		for i := 0; i < 10; i++ {
			fcs.Execute(state, 1.0/60.0)
		}
		
		// Measure performance
		iterations := 1000
		start := time.Now()
		
		for i := 0; i < iterations; i++ {
			fcs.Execute(state, 1.0/60.0)
		}
		
		elapsed := time.Since(start)
		avgTime := elapsed.Seconds() / float64(iterations)
		
		// Should execute in less than 1ms on average
		if avgTime > 0.001 {
			t.Errorf("Basic FCS too slow: %.3f ms average", avgTime*1000)
		}
		
		t.Logf("Basic FCS: %.3f ms average execution time", avgTime*1000)
	})
	
	t.Run("Realistic FCS Performance", func(t *testing.T) {
		fcs := CreateStandardP51DFlightControlSystem()
		state := NewAircraftState()
		
		// Warm up
		for i := 0; i < 10; i++ {
			fcs.Execute(state, 1.0/120.0)
		}
		
		// Measure performance
		iterations := 1000
		start := time.Now()
		
		for i := 0; i < iterations; i++ {
			fcs.Execute(state, 1.0/120.0)
		}
		
		elapsed := time.Since(start)
		avgTime := elapsed.Seconds() / float64(iterations)
		
		// Should execute in less than 5ms on average
		if avgTime > 0.005 {
			t.Errorf("Realistic FCS too slow: %.3f ms average", avgTime*1000)
		}
		
		t.Logf("Realistic FCS: %.3f ms average execution time", avgTime*1000)
	})
}
