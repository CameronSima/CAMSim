// Flight Control System Execution Engine
// Manages component execution with rate groups and channels

package main

import (
	"fmt"
	"sort"
	"time"
)

// RateGroupScheduler manages components that execute at a specific rate
type RateGroupScheduler struct {
	Name          string
	RateHz        float64
	Period        float64 // 1.0 / RateHz
	Components    []ComponentProcessor
	LastExecution time.Time
	ExecutionTime float64 // Accumulated execution time
	Enabled       bool
}

// NewRateGroupScheduler creates a new rate group scheduler
func NewRateGroupScheduler(name string, rateHz float64) *RateGroupScheduler {
	return &RateGroupScheduler{
		Name:          name,
		RateHz:        rateHz,
		Period:        1.0 / rateHz,
		Components:    make([]ComponentProcessor, 0),
		LastExecution: time.Time{}, // Zero time - will execute immediately
		Enabled:       true,
	}
}

// AddComponent adds a component to this rate group
func (rgs *RateGroupScheduler) AddComponent(component ComponentProcessor) {
	component.SetRateGroup(rgs.Name)
	rgs.Components = append(rgs.Components, component)
}

// ShouldExecute determines if this rate group should execute now
func (rgs *RateGroupScheduler) ShouldExecute(currentTime time.Time) bool {
	if !rgs.Enabled {
		return false
	}
	
	elapsed := currentTime.Sub(rgs.LastExecution).Seconds()
	return elapsed >= rgs.Period
}

// Execute runs all components in this rate group
func (rgs *RateGroupScheduler) Execute(properties *PropertyManager, dt float64) {
	if !rgs.Enabled {
		return
	}
	
	startTime := time.Now()
	
	// Execute all components in order
	for _, component := range rgs.Components {
		component.Execute(properties, dt)
	}
	
	rgs.LastExecution = startTime
	rgs.ExecutionTime = time.Since(startTime).Seconds()
}

// GetComponentCount returns the number of components in this rate group
func (rgs *RateGroupScheduler) GetComponentCount() int {
	return len(rgs.Components)
}

// =============================================================================
// CHANNEL PROCESSOR
// =============================================================================

// ChannelProcessor manages a logical group of components
type ChannelProcessor struct {
	Name       string
	Components []ComponentProcessor
	Enabled    bool
}

// NewChannelProcessor creates a new channel processor
func NewChannelProcessor(name string) *ChannelProcessor {
	return &ChannelProcessor{
		Name:       name,
		Components: make([]ComponentProcessor, 0),
		Enabled:    true,
	}
}

// AddComponent adds a component to this channel
func (cp *ChannelProcessor) AddComponent(component ComponentProcessor) {
	cp.Components = append(cp.Components, component)
}

// Execute runs all components in this channel
func (cp *ChannelProcessor) Execute(properties *PropertyManager, dt float64) {
	if !cp.Enabled {
		return
	}
	
	// Execute components in order
	for _, component := range cp.Components {
		component.Execute(properties, dt)
	}
}

// GetComponentCount returns the number of components in this channel
func (cp *ChannelProcessor) GetComponentCount() int {
	return len(cp.Components)
}

// =============================================================================
// FLIGHT CONTROL SYSTEM
// =============================================================================

// FlightControlSystem is the main FCS execution engine
type FlightControlSystem struct {
	Name         string
	Properties   *PropertyManager
	RateGroups   map[string]*RateGroupScheduler
	Channels     map[string]*ChannelProcessor
	Components   map[string]ComponentProcessor
	DefaultRate  float64 // Default execution rate (Hz)
	Enabled      bool
	
	// Statistics
	TotalExecutions int64
	TotalTime       float64
}

// NewFlightControlSystem creates a new flight control system
func NewFlightControlSystem(name string, defaultRateHz float64) *FlightControlSystem {
	fcs := &FlightControlSystem{
		Name:        name,
		Properties:  NewPropertyManager(),
		RateGroups:  make(map[string]*RateGroupScheduler),
		Channels:    make(map[string]*ChannelProcessor),
		Components:  make(map[string]ComponentProcessor),
		DefaultRate: defaultRateHz,
		Enabled:     true,
	}
	
	// Create default rate group
	defaultGroup := NewRateGroupScheduler("default", defaultRateHz)
	fcs.RateGroups["default"] = defaultGroup
	
	return fcs
}

// AddRateGroup adds a new rate group
func (fcs *FlightControlSystem) AddRateGroup(name string, rateHz float64) {
	fcs.RateGroups[name] = NewRateGroupScheduler(name, rateHz)
}

// AddChannel adds a new channel
func (fcs *FlightControlSystem) AddChannel(name string) *ChannelProcessor {
	channel := NewChannelProcessor(name)
	fcs.Channels[name] = channel
	return channel
}

// AddComponent adds a component to the system
func (fcs *FlightControlSystem) AddComponent(component ComponentProcessor) {
	// Register component
	fcs.Components[component.GetName()] = component
	
	// Assign to rate group
	rateGroupName := component.GetRateGroup()
	if rateGroupName == "" {
		rateGroupName = "default"
		component.SetRateGroup(rateGroupName)
	}
	
	// Find or create rate group
	if rateGroup, exists := fcs.RateGroups[rateGroupName]; exists {
		rateGroup.AddComponent(component)
	} else {
		// Component specifies non-existent rate group, use default
		fcs.RateGroups["default"].AddComponent(component)
	}
}

// Execute runs the flight control system for one time step
func (fcs *FlightControlSystem) Execute(state *AircraftState, dt float64) {
	if !fcs.Enabled {
		return
	}
	
	startTime := time.Now()
	
	// Update properties from aircraft state
	fcs.Properties.UpdateFromAircraftState(state)
	
	// Execute rate groups that are ready
	for _, rateGroup := range fcs.RateGroups {
		// For testing and high-frequency simulation, always execute but use simulation dt
		// In real-time applications, you might want to use the time-based scheduling
		rateGroup.Execute(fcs.Properties, dt)
	}
	
	// Apply results back to aircraft state
	fcs.Properties.ApplyToAircraftState(state)
	
	// Update statistics
	fcs.TotalExecutions++
	fcs.TotalTime += time.Since(startTime).Seconds()
}

// Reset resets all components and rate groups
func (fcs *FlightControlSystem) Reset() {
	for _, component := range fcs.Components {
		component.Reset()
	}
	
	for _, rateGroup := range fcs.RateGroups {
		rateGroup.LastExecution = time.Now()
	}
	
	fcs.TotalExecutions = 0
	fcs.TotalTime = 0.0
}

// GetComponent retrieves a component by name
func (fcs *FlightControlSystem) GetComponent(name string) ComponentProcessor {
	return fcs.Components[name]
}

// GetRateGroup retrieves a rate group by name
func (fcs *FlightControlSystem) GetRateGroup(name string) *RateGroupScheduler {
	return fcs.RateGroups[name]
}

// GetChannel retrieves a channel by name
func (fcs *FlightControlSystem) GetChannel(name string) *ChannelProcessor {
	return fcs.Channels[name]
}

// ListComponents returns all component names
func (fcs *FlightControlSystem) ListComponents() []string {
	var names []string
	for name := range fcs.Components {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

// ListRateGroups returns all rate group names
func (fcs *FlightControlSystem) ListRateGroups() []string {
	var names []string
	for name := range fcs.RateGroups {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

// GetStats returns execution statistics
func (fcs *FlightControlSystem) GetStats() map[string]interface{} {
	avgTime := 0.0
	if fcs.TotalExecutions > 0 {
		avgTime = fcs.TotalTime / float64(fcs.TotalExecutions)
	}
	
	return map[string]interface{}{
		"total_executions":     fcs.TotalExecutions,
		"total_time_seconds":   fcs.TotalTime,
		"average_time_seconds": avgTime,
		"components_count":     len(fcs.Components),
		"rate_groups_count":    len(fcs.RateGroups),
		"channels_count":       len(fcs.Channels),
	}
}

// String returns a string representation of the FCS
func (fcs *FlightControlSystem) String() string {
	stats := fcs.GetStats()
	
	return fmt.Sprintf("FlightControlSystem '%s' {\n"+
		"  Components: %d\n"+
		"  Rate Groups: %d\n"+
		"  Channels: %d\n"+
		"  Executions: %d\n"+
		"  Avg Time: %.6f ms\n"+
		"  Enabled: %t\n"+
		"}",
		fcs.Name,
		stats["components_count"],
		stats["rate_groups_count"],
		stats["channels_count"],
		stats["total_executions"],
		stats["average_time_seconds"].(float64)*1000,
		fcs.Enabled)
}

// =============================================================================
// FACTORY FUNCTIONS
// =============================================================================

// CreateStandardP51DFlightControlSystem creates a realistic P-51D FCS
func CreateStandardP51DFlightControlSystem() *FlightControlSystem {
	fcs := NewFlightControlSystem("P51D FCS", 120.0) // 120 Hz default rate
	
	// Add rate groups (as per JSBSim spec)
	fcs.AddRateGroup("high", 120.0)   // High-rate components
	fcs.AddRateGroup("medium", 40.0)  // Medium-rate components  
	fcs.AddRateGroup("low", 10.0)     // Low-rate components
	
	// Create channels
	pitchChannel := fcs.AddChannel("Pitch")
	rollChannel := fcs.AddChannel("Roll")
	yawChannel := fcs.AddChannel("Yaw")
	
	// =========================================================================
	// PITCH CHANNEL
	// =========================================================================
	
	// Elevator actuator with realistic P-51D characteristics
	elevatorActuator := NewActuatorComponent(
		"fcs/elevator-actuator",
		"fcs/elevator-cmd-norm",
		"fcs/elevator-pos-rad")
	elevatorActuator.SetRateLimit(2.5)      // 2.5 rad/sec max rate (realistic for P-51D)
	elevatorActuator.SetLag(0.06)           // 60ms lag (hydraulic/mechanical delay)
	elevatorActuator.SetHysteresis(0.02)    // 2% hysteresis band
	elevatorActuator.SetRateGroup("high")   // High-rate component
	
	// Elevator position gain (convert normalized to radians)
	elevatorGain := NewGainComponent(
		"fcs/elevator-gain",
		"fcs/elevator-pos-rad",
		"fcs/elevator-pos-deg",
		57.2958) // rad to deg conversion
	elevatorGain.SetRateGroup("medium")
	
	pitchChannel.AddComponent(elevatorActuator)
	pitchChannel.AddComponent(elevatorGain)
	fcs.AddComponent(elevatorActuator)
	fcs.AddComponent(elevatorGain)
	
	// =========================================================================
	// ROLL CHANNEL
	// =========================================================================
	
	// Left aileron actuator
	leftAileronActuator := NewActuatorComponent(
		"fcs/left-aileron-actuator",
		"fcs/aileron-cmd-norm",
		"fcs/left-aileron-pos-rad")
	leftAileronActuator.SetRateLimit(3.0)    // 3.0 rad/sec (ailerons are faster)
	leftAileronActuator.SetLag(0.04)         // 40ms lag
	leftAileronActuator.SetRateGroup("high")
	
	// Right aileron actuator (opposite sign)
	rightAileronGain := NewGainComponent(
		"fcs/right-aileron-gain",
		"fcs/aileron-cmd-norm",
		"fcs/right-aileron-cmd-norm",
		-1.0) // Opposite deflection
	
	rightAileronActuator := NewActuatorComponent(
		"fcs/right-aileron-actuator",
		"fcs/right-aileron-cmd-norm",
		"fcs/right-aileron-pos-rad")
	rightAileronActuator.SetRateLimit(3.0)
	rightAileronActuator.SetLag(0.04)
	rightAileronActuator.SetRateGroup("high")
	
	rollChannel.AddComponent(leftAileronActuator)
	rollChannel.AddComponent(rightAileronGain)
	rollChannel.AddComponent(rightAileronActuator)
	fcs.AddComponent(leftAileronActuator)
	fcs.AddComponent(rightAileronGain)
	fcs.AddComponent(rightAileronActuator)
	
	// =========================================================================
	// YAW CHANNEL
	// =========================================================================
	
	// Rudder actuator
	rudderActuator := NewActuatorComponent(
		"fcs/rudder-actuator",
		"fcs/rudder-cmd-norm",
		"fcs/rudder-pos-rad")
	rudderActuator.SetRateLimit(2.0)      // 2.0 rad/sec
	rudderActuator.SetLag(0.08)           // 80ms lag (rudder is slower)
	rudderActuator.SetHysteresis(0.03)    // 3% hysteresis
	rudderActuator.SetRateGroup("high")
	
	yawChannel.AddComponent(rudderActuator)
	fcs.AddComponent(rudderActuator)
	
	return fcs
}

// CreateBasicFlightControlSystem creates a simple FCS for testing
func CreateBasicFlightControlSystem() *FlightControlSystem {
	fcs := NewFlightControlSystem("Basic FCS", 60.0)
	
	// Simple direct pass-through components
	elevatorPass := NewGainComponent("elevator-pass", "fcs/elevator-cmd-norm", "fcs/elevator-pos-rad", 1.0)
	aileronPass := NewGainComponent("aileron-pass", "fcs/aileron-cmd-norm", "fcs/left-aileron-pos-rad", 1.0)
	rudderPass := NewGainComponent("rudder-pass", "fcs/rudder-cmd-norm", "fcs/rudder-pos-rad", 1.0)
	
	fcs.AddComponent(elevatorPass)
	fcs.AddComponent(aileronPass)
	fcs.AddComponent(rudderPass)
	
	return fcs
}
