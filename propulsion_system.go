// Propulsion System Implementation
// Based on actual P-51D JSBSim configuration values

package main

import (
	"fmt"
	"math"
)

// PropulsionSystem manages engine and propeller for the P-51D
type PropulsionSystem struct {
	Engine     *PistonEngine
	Propeller  *Propeller
	FuelSystem *FuelSystem
	Properties *PropertyManager
	
	// Configuration from JSBSim XML
	MaxThrust        float64 // 200 lbs at reference conditions (from XML line 635)
	ReferenceRPM     float64 // 1260 RPM (from XML line 632)
	ReferenceMAP     float64 // 81 inHg (from XML line 634)
	RunningFactor    float64 // 0.3 (off) or 1.0 (running)
	
	// Engine position from XML (line 504-508, 520-524)
	Position    Vector3 // 36 inches from datum
	Orientation Vector3 // Roll: -4.0°, Pitch: 2.5°, Yaw: 0°
}

// PistonEngine represents the Packard V-1650-7 engine
type PistonEngine struct {
	Name             string
	IsRunning        bool
	
	// Current state
	RPM              float64 // Current propeller RPM
	ManifoldPressure float64 // Current MAP in inHg
	ThrottlePosition float64 // 0.0 to 1.0
	
	// Performance characteristics (will be enhanced with JSBSim tables later)
	MaxRPM           float64 // Maximum RPM
	MaxMAP           float64 // Maximum manifold pressure
	IdleRPM          float64 // Idle RPM
	IdleMAP          float64 // Idle manifold pressure
}

// Propeller represents the P51 propeller
type Propeller struct {
	Name      string
	Diameter  float64 // From JSBSim propeller file (will extract later)
	
	// Current state
	RPM       float64 // Propeller RPM
	Thrust    float64 // Current thrust in lbs
	Torque    float64 // Current torque
	
	// Propeller-induced velocity for aerodynamic effects
	InducedVelocity float64 // prop-induced-velocity_fps
}

// FuelSystem manages the 5 fuel tanks from JSBSim config
type FuelSystem struct {
	Tanks         []*FuelTank
	TotalCapacity float64 // Total capacity in lbs
	TotalContents float64 // Current fuel in lbs
	FuelFlow      float64 // Current consumption lbs/hr
}

// FuelTank represents individual fuel tanks from JSBSim config
type FuelTank struct {
	Number   int     // Tank number (0-4 from XML)
	Type     string  // FUEL type
	Position Vector3 // Tank position from XML
	Capacity float64 // Capacity in lbs (from XML)
	Contents float64 // Current contents in lbs
	Priority int     // Feed priority
}

// NewPropulsionSystem creates a P-51D propulsion system with JSBSim values
func NewPropulsionSystem() *PropulsionSystem {
	ps := &PropulsionSystem{
		// Values extracted directly from JSBSim XML
		MaxThrust:     200.0, // lbs (line 635)
		ReferenceRPM:  1260.0, // RPM (line 632 comment)
		ReferenceMAP:  81.0,   // inHg (line 634 comment)
		RunningFactor: 0.3,    // Engine off initially
		
		// Engine position from XML (converted from inches to meters)
		Position: Vector3{
			X: 36.0 * 0.0254, // 36 inches to meters
			Y: 0.0,
			Z: 0.0,
		},
		Orientation: Vector3{
			X: -4.0 * DEG_TO_RAD, // Roll: -4.0°
			Y: 2.5 * DEG_TO_RAD,  // Pitch: 2.5°
			Z: 0.0,               // Yaw: 0°
		},
	}
	
	// Create engine with realistic characteristics
	ps.Engine = &PistonEngine{
		Name:             "Packard-V-1650-7",
		IsRunning:        false,
		RPM:              0.0,
		ManifoldPressure: 29.92, // Sea level atmospheric pressure
		ThrottlePosition: 0.0,
		
		// Realistic limits for P-51D (will be refined with JSBSim data)
		MaxRPM:  3000.0, // Typical max RPM for Merlin engine
		MaxMAP:   61.0,  // Typical max MAP for supercharged Merlin
		IdleRPM:  800.0, // Typical idle RPM
		IdleMAP:  15.0,  // Typical idle MAP
	}
	
	// Create propeller
	ps.Propeller = &Propeller{
		Name:            "P51prop",
		Diameter:        11.2, // feet (typical P-51D prop diameter)
		RPM:             0.0,
		Thrust:          0.0,
		Torque:          0.0,
		InducedVelocity: 0.0,
	}
	
	// Create fuel system with actual tank data from XML
	ps.FuelSystem = ps.createFuelSystemFromXML()
	
	return ps
}

// createFuelSystemFromXML creates fuel system based on XML tank definitions
func (ps *PropulsionSystem) createFuelSystemFromXML() *FuelSystem {
	fs := &FuelSystem{
		Tanks: make([]*FuelTank, 5), // 5 tanks from XML
	}
	
	// Tank data extracted from JSBSim XML lines 539-593
	tankConfigs := []struct {
		number   int
		position Vector3
		capacity float64
		contents float64
		priority int
	}{
		// Tank 0: Left wing (lines 539-549)
		{0, Vector3{X: 106 * 0.0254, Y: -80 * 0.0254, Z: -9.675 * 0.0254}, 553.84, 396.0, 1},
		// Tank 1: Right wing (lines 550-560)
		{1, Vector3{X: 106 * 0.0254, Y: 80 * 0.0254, Z: -9.675 * 0.0254}, 553.84, 396.0, 1},
		// Tank 2: Fuselage (lines 561-571)
		{2, Vector3{X: 160 * 0.0254, Y: 0.0, Z: -3.0 * 0.0254}, 511.7, 0.0, 1},
		// Tank 3: Left drop tank (lines 572-582)
		{3, Vector3{X: 97.5 * 0.0254, Y: -198 * 0.0254, Z: -25 * 0.0254}, 451.5, 0.0, 1},
		// Tank 4: Right drop tank (lines 583-593)
		{4, Vector3{X: 97.5 * 0.0254, Y: 198 * 0.0254, Z: -25 * 0.0254}, 451.5, 0.0, 1},
	}
	
	for i, config := range tankConfigs {
		fs.Tanks[i] = &FuelTank{
			Number:   config.number,
			Type:     "AVGAS",
			Position: config.position,
			Capacity: config.capacity,
			Contents: config.contents,
			Priority: config.priority,
		}
		fs.TotalCapacity += config.capacity
		fs.TotalContents += config.contents
	}
	
	return fs
}

// Update updates the propulsion system state
func (ps *PropulsionSystem) Update(throttleInput float64, dt float64) {
	// Update throttle position
	ps.Engine.ThrottlePosition = math.Max(0.0, math.Min(1.0, throttleInput))
	
	// Update engine state
	ps.updateEngine(dt)
	
	// Update propeller state
	ps.updatePropeller(dt)
	
	// Update fuel consumption
	ps.updateFuelConsumption(dt)
	
	// Set running factor based on engine state
	if ps.Engine.IsRunning {
		ps.RunningFactor = 1.0
	} else {
		ps.RunningFactor = 0.3 // Windmilling factor
	}
}

// updateEngine updates engine RPM and manifold pressure based on throttle
func (ps *PropulsionSystem) updateEngine(dt float64) {
	if !ps.Engine.IsRunning && ps.Engine.ThrottlePosition > 0.1 {
		ps.Engine.IsRunning = true // Simple startup logic
	}
	
	if ps.Engine.IsRunning {
		// Linear interpolation between idle and max (simplified)
		throttle := ps.Engine.ThrottlePosition
		ps.Engine.RPM = ps.Engine.IdleRPM + throttle*(ps.Engine.MaxRPM-ps.Engine.IdleRPM)
		ps.Engine.ManifoldPressure = ps.Engine.IdleMAP + throttle*(ps.Engine.MaxMAP-ps.Engine.IdleMAP)
	} else {
		ps.Engine.RPM = 0.0
		ps.Engine.ManifoldPressure = 29.92 // Atmospheric pressure
	}
	
	// Copy engine RPM to propeller
	ps.Propeller.RPM = ps.Engine.RPM
}

// updatePropeller calculates propeller thrust using JSBSim formula
func (ps *PropulsionSystem) updatePropeller(dt float64) {
	// Use exact formula from JSBSim XML (lines 629-636)
	// Thrust = running_factor × (propeller_rpm / 1260) × (map_inhg / 81) × 200 lbs
	
	rpmFactor := ps.Propeller.RPM / ps.ReferenceRPM
	mapFactor := ps.Engine.ManifoldPressure / ps.ReferenceMAP
	
	ps.Propeller.Thrust = ps.RunningFactor * rpmFactor * mapFactor * ps.MaxThrust
	
	// Ensure thrust is not negative
	ps.Propeller.Thrust = math.Max(0.0, ps.Propeller.Thrust)
	
	// Calculate propeller-induced velocity (simplified)
	// This affects aerodynamic calculations
	if ps.Propeller.Thrust > 0 {
		ps.Propeller.InducedVelocity = math.Sqrt(ps.Propeller.Thrust / 0.5) // Simplified
	} else {
		ps.Propeller.InducedVelocity = 0.0
	}
}

// updateFuelConsumption calculates fuel burn based on power output
func (ps *PropulsionSystem) updateFuelConsumption(dt float64) {
	if ps.Engine.IsRunning && ps.Propeller.Thrust > 0 {
		// Simplified fuel flow: roughly 0.4-0.6 lbs/hr per HP
		// Estimate HP from thrust (very rough approximation)
		estimatedHP := ps.Propeller.Thrust * 0.75 // Rough conversion
		ps.FuelSystem.FuelFlow = estimatedHP * 0.5 // lbs/hr per HP
		
		// Consume fuel from tanks
		fuelBurn := ps.FuelSystem.FuelFlow * dt / 3600.0 // Convert hr to seconds
		ps.consumeFuel(fuelBurn)
	} else {
		ps.FuelSystem.FuelFlow = 0.0
	}
}

// consumeFuel removes fuel from tanks based on priority
func (ps *PropulsionSystem) consumeFuel(fuelBurn float64) {
	remainingBurn := fuelBurn
	
	// Consume from tanks with available fuel
	for _, tank := range ps.FuelSystem.Tanks {
		if remainingBurn <= 0 || tank.Contents <= 0 {
			continue
		}
		
		burnFromTank := math.Min(remainingBurn, tank.Contents)
		tank.Contents -= burnFromTank
		remainingBurn -= burnFromTank
		ps.FuelSystem.TotalContents -= burnFromTank
	}
}

// GetThrust returns current thrust in Newtons (converted from lbs)
func (ps *PropulsionSystem) GetThrust() float64 {
	return ps.Propeller.Thrust * 4.448222 // Convert lbs to Newtons
}

// GetTorque returns current torque in N⋅m
func (ps *PropulsionSystem) GetTorque() float64 {
	// Simple torque calculation from thrust and prop diameter
	if ps.Propeller.RPM > 0 {
		// Torque = Power / Angular velocity
		power := ps.Propeller.Thrust * ps.Propeller.InducedVelocity // Rough power estimate
		angularVel := ps.Propeller.RPM * 2.0 * math.Pi / 60.0       // Convert RPM to rad/s
		if angularVel > 0 {
			ps.Propeller.Torque = power / angularVel
		}
	} else {
		ps.Propeller.Torque = 0.0
	}
	
	return ps.Propeller.Torque
}

// UpdateProperties updates JSBSim-compatible properties
func (ps *PropulsionSystem) UpdateProperties(properties *PropertyManager) {
	// Set propulsion properties that JSBSim functions expect
	properties.Set("propulsion/engine/set-running", boolToFloat(ps.Engine.IsRunning))
	properties.Set("propulsion/engine/propeller-rpm", ps.Propeller.RPM)
	properties.Set("propulsion/engine/map-inhg", ps.Engine.ManifoldPressure)
	properties.Set("propulsion/engine/thrust-lbs", ps.Propeller.Thrust)
	properties.Set("propulsion/engine/prop-induced-velocity_fps", ps.Propeller.InducedVelocity)
	
	// External reactions (thrust force)
	properties.Set("external_reactions/exhaust-thrust/magnitude", ps.Propeller.Thrust)
	
	// Fuel system properties
	properties.Set("propulsion/total-fuel-lbs", ps.FuelSystem.TotalContents)
	properties.Set("propulsion/fuel-flow-rate_pph", ps.FuelSystem.FuelFlow)
}

// String returns a string representation of the propulsion system
func (ps *PropulsionSystem) String() string {
	return fmt.Sprintf("P-51D Propulsion System {\n"+
		"  Engine: %s (Running: %t)\n"+
		"  RPM: %.0f / %.0f\n"+
		"  MAP: %.1f inHg\n"+
		"  Throttle: %.1f%%\n"+
		"  Thrust: %.1f lbs (%.0f N)\n"+
		"  Fuel: %.1f / %.1f lbs\n"+
		"  Fuel Flow: %.1f lbs/hr\n"+
		"}",
		ps.Engine.Name,
		ps.Engine.IsRunning,
		ps.Engine.RPM, ps.Engine.MaxRPM,
		ps.Engine.ManifoldPressure,
		ps.Engine.ThrottlePosition*100,
		ps.Propeller.Thrust, ps.GetThrust(),
		ps.FuelSystem.TotalContents, ps.FuelSystem.TotalCapacity,
		ps.FuelSystem.FuelFlow)
}

// Helper function to convert bool to float for properties
func boolToFloat(b bool) float64 {
	if b {
		return 1.0
	}
	return 0.0
}
