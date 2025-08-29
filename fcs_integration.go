// Flight Control System Integration
// Connects the FCS to the main flight dynamics simulation

package main

import (
	"fmt"
	"math"
	"os"
	"strings"
)

// FlightDynamicsEngineWithFCS extends the basic flight dynamics engine with FCS
type FlightDynamicsEngineWithFCS struct {
	*FlightDynamicsEngine  // Embed the basic engine
	FCS                    *FlightControlSystem
	UseRealisticControls   bool // Use FCS vs direct mapping
}

// NewFlightDynamicsEngineWithFCS creates a flight dynamics engine with FCS
func NewFlightDynamicsEngineWithFCS(config *JSBSimConfig, useRealisticFCS bool) (*FlightDynamicsEngineWithFCS, error) {
	// Create base engine with RK4 integrator
	integrator := &RungeKutta4Integrator{}
	baseEngine := NewFlightDynamicsEngine(config, integrator)
	
	// Create FCS
	var fcs *FlightControlSystem
	if useRealisticFCS {
		fcs = CreateStandardP51DFlightControlSystem()
	} else {
		fcs = CreateBasicFlightControlSystem()
	}
	
	return &FlightDynamicsEngineWithFCS{
		FlightDynamicsEngine: baseEngine,
		FCS:                  fcs,
		UseRealisticControls: useRealisticFCS,
	}, nil
}

// RunSimulationStepWithFCS runs one simulation step with flight control processing
func (engine *FlightDynamicsEngineWithFCS) RunSimulationStepWithFCS(
	state *AircraftState, 
	dt float64) (*AircraftState, *StateDerivatives, error) {
	
	// 1. Process pilot inputs through flight control system
	engine.FCS.Execute(state, dt)
	
	// 2. Run the base flight dynamics with FCS-processed control surfaces
	newState, err := engine.FlightDynamicsEngine.Step(state, dt)
	if err != nil {
		return nil, nil, err
	}
	
	// Calculate derivatives for compatibility (we don't have them from Step method)
	derivatives := &StateDerivatives{
		PositionDot:     Vector3{}, // Would need to calculate from velocity
		VelocityDot:     Vector3{}, // Would need from forces
		OrientationDot:  Quaternion{}, // Would need from angular rates
		AngularRateDot:  Vector3{}, // Would need from moments
		AltitudeDot:     0.0, // Would calculate from velocity Z
	}
	
	return newState, derivatives, nil
}

// SetControlInputs applies pilot control inputs to the FCS property system
func (engine *FlightDynamicsEngineWithFCS) SetControlInputs(controls ControlInputs) {
	// Set pilot command properties (these feed into the FCS)
	engine.FCS.Properties.Set("fcs/aileron-cmd-norm", controls.Aileron)
	engine.FCS.Properties.Set("fcs/elevator-cmd-norm", controls.Elevator)
	engine.FCS.Properties.Set("fcs/rudder-cmd-norm", controls.Rudder)
	engine.FCS.Properties.Set("fcs/throttle-cmd-norm", controls.Throttle)
	engine.FCS.Properties.Set("fcs/flap-cmd-norm", controls.Flaps)
}

// SetControlInputsOnState applies control inputs to aircraft state (for integration)
func (engine *FlightDynamicsEngineWithFCS) SetControlInputsOnState(state *AircraftState, controls ControlInputs) {
	// Update aircraft state controls
	state.Controls = controls
	
	// Also set FCS properties
	engine.SetControlInputs(controls)
}

// GetControlSurfacePositions retrieves actual control surface positions from FCS
func (engine *FlightDynamicsEngineWithFCS) GetControlSurfacePositions() map[string]float64 {
	return map[string]float64{
		"elevator":      engine.FCS.Properties.Get("fcs/elevator-pos-rad"),
		"left_aileron":  engine.FCS.Properties.Get("fcs/left-aileron-pos-rad"),
		"right_aileron": engine.FCS.Properties.Get("fcs/right-aileron-pos-rad"),
		"rudder":        engine.FCS.Properties.Get("fcs/rudder-pos-rad"),
	}
}

// GetFCSStatus returns detailed FCS status information
func (engine *FlightDynamicsEngineWithFCS) GetFCSStatus() map[string]interface{} {
	status := engine.FCS.GetStats()
	
	// Add control surface positions
	status["control_surfaces"] = engine.GetControlSurfacePositions()
	
	// Add pilot commands
	status["pilot_commands"] = map[string]float64{
		"aileron":  engine.FCS.Properties.Get("fcs/aileron-cmd-norm"),
		"elevator": engine.FCS.Properties.Get("fcs/elevator-cmd-norm"),
		"rudder":   engine.FCS.Properties.Get("fcs/rudder-cmd-norm"),
		"throttle": engine.FCS.Properties.Get("fcs/throttle-cmd-norm"),
	}
	
	// Add rate group information
	rateGroupInfo := make(map[string]interface{})
	for name, rg := range engine.FCS.RateGroups {
		rateGroupInfo[name] = map[string]interface{}{
			"rate_hz":         rg.RateHz,
			"component_count": rg.GetComponentCount(),
			"execution_time":  rg.ExecutionTime,
			"enabled":         rg.Enabled,
		}
	}
	status["rate_groups"] = rateGroupInfo
	
	return status
}

// =============================================================================
// ENHANCED CONTROL SURFACE MAPPING
// =============================================================================

// UpdateControlSurfacesFromFCS updates aircraft state control surfaces from FCS outputs
func (engine *FlightDynamicsEngineWithFCS) UpdateControlSurfacesFromFCS(state *AircraftState) {
	if engine.UseRealisticControls {
		// Use FCS-processed values (with actuator dynamics)
		state.ControlSurfaces.Elevator = engine.FCS.Properties.Get("fcs/elevator-pos-rad")
		state.ControlSurfaces.AileronLeft = engine.FCS.Properties.Get("fcs/left-aileron-pos-rad")
		state.ControlSurfaces.AileronRight = engine.FCS.Properties.Get("fcs/right-aileron-pos-rad")
		state.ControlSurfaces.Rudder = engine.FCS.Properties.Get("fcs/rudder-pos-rad")
	} else {
		// Use direct mapping (bypass FCS for comparison)
		state.ControlSurfaces.Elevator = state.Controls.Elevator * 25.0 * DEG_TO_RAD
		state.ControlSurfaces.AileronLeft = state.Controls.Aileron * 30.0 * DEG_TO_RAD
		state.ControlSurfaces.AileronRight = -state.Controls.Aileron * 30.0 * DEG_TO_RAD
		state.ControlSurfaces.Rudder = state.Controls.Rudder * 30.0 * DEG_TO_RAD
	}
}

// =============================================================================
// DEMONSTRATION FUNCTIONS
// =============================================================================

// DemoFlightControlSystem demonstrates the FCS functionality
func DemoFlightControlSystem() {
	fmt.Println("\n" + strings.Repeat("=", 60))
	fmt.Println("🎮 FLIGHT CONTROL SYSTEM DEMONSTRATION")
	fmt.Println(strings.Repeat("=", 60))
	
	// Create aircraft state
	state := NewAircraftState()
	state.Altitude = 3000.0 // 3000m altitude
	state.Position.Z = -3000.0 // NED coordinates
	state.Velocity = Vector3{X: 100.0, Y: 0.0, Z: 0.0} // 100 m/s forward
	state.UpdateAtmosphere()
	state.UpdateDerivedParameters()
	
	// Create FCS engines for comparison
	basicFCS := CreateBasicFlightControlSystem()
	realisticFCS := CreateStandardP51DFlightControlSystem()
	
	fmt.Printf("\n📊 FCS Comparison:\n")
	fmt.Printf("   Basic FCS:     %d components, %d rate groups\n", 
		len(basicFCS.Components), len(basicFCS.RateGroups))
	fmt.Printf("   Realistic FCS: %d components, %d rate groups\n", 
		len(realisticFCS.Components), len(realisticFCS.RateGroups))
	
	// Test control inputs
	testInputs := []ControlInputs{
		{Aileron: 0.0, Elevator: 0.0, Rudder: 0.0, Throttle: 0.5},    // Neutral
		{Aileron: 0.5, Elevator: 0.2, Rudder: 0.1, Throttle: 0.8},    // Right bank, climb
		{Aileron: -0.3, Elevator: -0.1, Rudder: -0.05, Throttle: 0.3}, // Left bank, descend
	}
	
	fmt.Printf("\n🎯 Control Response Comparison:\n")
	fmt.Printf("%-15s %-12s %-12s %-12s %-12s %-12s\n", 
		"Input", "Basic Elev", "Real Elev", "Basic Ail", "Real Ail", "Real Rud")
	fmt.Printf("%s\n", strings.Repeat("-", 80))
	
	dt := 1.0/60.0 // 60 Hz simulation
	
	for i, input := range testInputs {
		// Apply inputs to aircraft state (so they persist through FCS.Execute)
		state.Controls = input
		
		// Execute multiple steps to see actuator dynamics
		for step := 0; step < 5; step++ {
			basicFCS.Execute(state, dt)
			realisticFCS.Execute(state, dt)
		}
		
		// Get outputs
		basicElev := basicFCS.Properties.Get("fcs/elevator-pos-rad") * RAD_TO_DEG
		realElev := realisticFCS.Properties.Get("fcs/elevator-pos-rad") * RAD_TO_DEG
		basicAil := basicFCS.Properties.Get("fcs/left-aileron-pos-rad") * RAD_TO_DEG
		realAil := realisticFCS.Properties.Get("fcs/left-aileron-pos-rad") * RAD_TO_DEG
		realRud := realisticFCS.Properties.Get("fcs/rudder-pos-rad") * RAD_TO_DEG
		
		fmt.Printf("Test %d:       %8.1f°   %8.1f°   %8.1f°   %8.1f°   %8.1f°\n", 
			i+1, basicElev, realElev, basicAil, realAil, realRud)
	}
	
	// Demonstrate actuator dynamics
	fmt.Printf("\n⚡ Actuator Dynamics Test (Step Response):\n")
	
	// Reset realistic FCS
	realisticFCS.Reset()
	
	// Apply step input and track response
	state.Controls.Elevator = 1.0 // Full elevator command
	
	fmt.Printf("Time(s)  Command(°)  Response(°)  Rate(°/s)\n")
	fmt.Printf("%s\n", strings.Repeat("-", 45))
	
	prevResponse := 0.0
	for step := 0; step < 20; step++ {
		time := float64(step) * dt
		realisticFCS.Execute(state, dt)
		
		command := 1.0 * 25.0 // Full deflection in degrees
		response := realisticFCS.Properties.Get("fcs/elevator-pos-rad") * RAD_TO_DEG
		rate := (response - prevResponse) / dt
		
		if step%3 == 0 { // Print every 3rd step
			fmt.Printf("%6.3f   %9.1f   %10.1f   %8.1f\n", 
				time, command, response, rate)
		}
		
		prevResponse = response
	}
	
	// Show final statistics
	fmt.Printf("\n📈 Performance Statistics:\n")
	basicStats := basicFCS.GetStats()
	realisticStats := realisticFCS.GetStats()
	
	fmt.Printf("   Basic FCS:     %.3f ms avg execution time\n", 
		basicStats["average_time_seconds"].(float64)*1000)
	fmt.Printf("   Realistic FCS: %.3f ms avg execution time\n", 
		realisticStats["average_time_seconds"].(float64)*1000)
	
	fmt.Printf("\n✅ Flight Control System Demo Complete!\n")
}

// DemoRealisticVsDirectControl compares realistic FCS vs direct control mapping
func DemoRealisticVsDirectControl() {
	fmt.Println("\n" + strings.Repeat("=", 60))
	fmt.Println("⚖️  REALISTIC vs DIRECT CONTROL COMPARISON")
	fmt.Println(strings.Repeat("=", 60))
	
	// Load P-51D configuration
	file, err := os.Open("/Users/cameronsima/dev/camSIM_go/aircraft/p51d-jsbsim.xml")
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
	
	// Create both engine types
	directEngine, err := NewFlightDynamicsEngineWithFCS(config, false)
	if err != nil {
		fmt.Printf("❌ Error creating direct engine: %v\n", err)
		return
	}
	
	realisticEngine, err := NewFlightDynamicsEngineWithFCS(config, true)
	if err != nil {
		fmt.Printf("❌ Error creating realistic engine: %v\n", err)
		return
	}
	
	// Create identical initial states
	directState := NewAircraftState()
	realisticState := NewAircraftState()
	
	// Set identical initial conditions
	initialConditions := func(state *AircraftState) {
		state.Altitude = 3000.0
		state.Position.Z = -3000.0
		state.Velocity = Vector3{X: 120.0, Y: 0.0, Z: 0.0} // 120 m/s
		state.UpdateAtmosphere()
		state.UpdateDerivedParameters()
	}
	
	initialConditions(directState)
	initialConditions(realisticState)
	
	// Test scenario: Aggressive elevator input
	fmt.Printf("\n🎯 Test Scenario: Aggressive Elevator Pull (1.0 → 0.0 → -0.5)\n")
	fmt.Printf("Time(s)  Direct Elev(°)  Real Elev(°)  Difference(°)\n")
	fmt.Printf("%s\n", strings.Repeat("-", 50))
	
	dt := 1.0/120.0 // 120 Hz
	
	for step := 0; step < 240; step++ { // 2 seconds
		time := float64(step) * dt
		
		// Define elevator command profile
		var elevatorCmd float64
		if time < 0.5 {
			elevatorCmd = 1.0 // Full up
		} else if time < 1.0 {
			elevatorCmd = 0.0 // Neutral
		} else {
			elevatorCmd = -0.5 // Half down
		}
		
		// Apply controls to both engines
		controls := ControlInputs{
			Elevator: elevatorCmd,
			Aileron:  0.0,
			Rudder:   0.0,
			Throttle: 0.8,
		}
		
		directEngine.SetControlInputsOnState(directState, controls)
		realisticEngine.SetControlInputsOnState(realisticState, controls)
		
		// Run simulation steps
		directState, _, _ = directEngine.RunSimulationStepWithFCS(directState, dt)
		realisticState, _, _ = realisticEngine.RunSimulationStepWithFCS(realisticState, dt)
		
		// Print comparison every 24 steps (0.2s intervals)
		if step%24 == 0 {
			directElev := directState.ControlSurfaces.Elevator * RAD_TO_DEG
			realElev := realisticState.ControlSurfaces.Elevator * RAD_TO_DEG
			diff := math.Abs(directElev - realElev)
			
			fmt.Printf("%6.1f   %12.1f   %10.1f   %11.1f\n", 
				time, directElev, realElev, diff)
		}
	}
	
	// Show final FCS status
	fmt.Printf("\n📊 Final FCS Status:\n")
	realisticStatus := realisticEngine.GetFCSStatus()
	
	fmt.Printf("   Total Executions: %d\n", realisticStatus["total_executions"])
	fmt.Printf("   Average Exec Time: %.3f ms\n", 
		realisticStatus["average_time_seconds"].(float64)*1000)
	
	rateGroups := realisticStatus["rate_groups"].(map[string]interface{})
	for name, info := range rateGroups {
		rateInfo := info.(map[string]interface{})
		fmt.Printf("   Rate Group '%s': %.0f Hz, %d components, %.3f ms\n",
			name,
			rateInfo["rate_hz"].(float64),
			rateInfo["component_count"].(int),
			rateInfo["execution_time"].(float64)*1000)
	}
	
	fmt.Printf("\n✅ Realistic vs Direct Control Comparison Complete!\n")
	fmt.Printf("💡 Key Insight: Realistic FCS adds actuator lag and rate limiting\n")
	fmt.Printf("   This prevents instantaneous control surface movement and adds realism.\n")
}
