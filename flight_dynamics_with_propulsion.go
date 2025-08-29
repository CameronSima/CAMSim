// Flight Dynamics Engine with Integrated Propulsion System
// Combines flight dynamics, flight controls, and propulsion into a unified system

package main

import (
	"fmt"
	"math"
)

// FlightDynamicsEngineWithPropulsion integrates all major aircraft systems
type FlightDynamicsEngineWithPropulsion struct {
	*FlightDynamicsEngineWithFCS // Embed FCS-enabled flight dynamics
	Propulsion                   *PropulsionSystem
	UseRealisticPropulsion       bool // Use detailed propulsion vs simplified
	
	// Integration state
	LastFuelWeight float64 // Track fuel weight changes
}

// NewFlightDynamicsEngineWithPropulsion creates a complete flight dynamics system
func NewFlightDynamicsEngineWithPropulsion(config *JSBSimConfig, useRealisticFCS, useRealisticPropulsion bool) (*FlightDynamicsEngineWithPropulsion, error) {
	// Create the FCS-enabled flight dynamics engine
	fcsEngine, err := NewFlightDynamicsEngineWithFCS(config, useRealisticFCS)
	if err != nil {
		return nil, fmt.Errorf("failed to create FCS engine: %v", err)
	}
	
	// Create propulsion system
	propulsion := NewPropulsionSystem()
	
	// Initialize fuel weight tracking
	initialFuelWeight := propulsion.FuelSystem.TotalContents
	
	engine := &FlightDynamicsEngineWithPropulsion{
		FlightDynamicsEngineWithFCS: fcsEngine,
		Propulsion:                  propulsion,
		UseRealisticPropulsion:      useRealisticPropulsion,
		LastFuelWeight:              initialFuelWeight,
	}
	
	// Note: True RK4 with dynamics re-evaluation is available but disabled for stability
	// The improved RK4 approximation in the base integrator provides good results
	// while maintaining numerical stability
	
	return engine, nil
}

// RunSimulationStepWithPropulsion runs one complete simulation step
func (engine *FlightDynamicsEngineWithPropulsion) RunSimulationStepWithPropulsion(
	state *AircraftState, 
	dt float64) (*AircraftState, *StateDerivatives, error) {
	
	// 1. Update propulsion system with throttle input
	engine.updatePropulsionSystem(state, dt)
	
	// 2. Update aircraft weight based on fuel consumption
	engine.updateAircraftWeight(state)
	
	// 3. Update property manager with propulsion data
	engine.updatePropulsionProperties(state)
	
	// 4. Run flight control system processing and get base derivatives
	_, derivatives, err := engine.FlightDynamicsEngineWithFCS.RunSimulationStepWithFCS(state, dt)
	if err != nil {
		return nil, nil, err
	}
	
	// 5. Add propulsion forces and moments to the derivatives
	propulsionForces := engine.calculatePropulsionForces(state)
	propulsionMoments := engine.calculatePropulsionMoments(state)
	
	// Approximate aircraft mass (would be better to have in state)
	approxMass := 11600.0 * 0.453592 // P-51D loaded weight in kg
	
	// Add propulsion forces to existing aerodynamic forces
	derivatives.VelocityDot.X += propulsionForces.X / approxMass
	derivatives.VelocityDot.Y += propulsionForces.Y / approxMass
	derivatives.VelocityDot.Z += propulsionForces.Z / approxMass
	
	// Add propulsion moments to existing aerodynamic moments
	derivatives.AngularRateDot.X += propulsionMoments.X
	derivatives.AngularRateDot.Y += propulsionMoments.Y
	derivatives.AngularRateDot.Z += propulsionMoments.Z
	
	// Re-integrate with combined forces (propulsion + aerodynamics)
	finalState := engine.Integrator.Integrate(state, derivatives, dt)
	
	// 6. Apply constraints and validate
	engine.applyConstraints(finalState)
	engine.validateState(finalState)
	
	return finalState, derivatives, nil
}

// calculateDynamicsAtState calculates state derivatives for a given state (used by true RK4)
func (engine *FlightDynamicsEngineWithPropulsion) calculateDynamicsAtState(state *AircraftState) (*StateDerivatives, error) {
	// Create a temporary copy of the propulsion system to avoid state corruption
	// during intermediate RK4 evaluations
	tempPropulsion := *engine.Propulsion
	tempEngine := *engine.Propulsion.Engine
	tempPropeller := *engine.Propulsion.Propeller
	tempPropulsion.Engine = &tempEngine
	tempPropulsion.Propeller = &tempPropeller
	
	// Update temporary propulsion system for this state
	tempPropulsion.Update(state.Controls.Throttle, 0.01)
	
	// Calculate aerodynamic forces and moments
	components, err := engine.FlightDynamicsEngine.Calculator.CalculateForcesMoments(state)
	if err != nil {
		return nil, err
	}
	
	// Get base derivatives from aerodynamics
	derivatives := engine.FlightDynamicsEngine.Calculator.CalculateStateDerivatives(state, components)
	
	// Calculate propulsion forces using temporary system
	thrust := tempPropulsion.GetThrust()
	
	// Transform thrust vector by engine orientation (simplified for stability)
	thrustX := thrust // Pure forward thrust for numerical stability
	thrustY := 0.0    // No side thrust
	thrustZ := 0.0    // No vertical thrust
	
	propulsionForces := Vector3{X: thrustX, Y: thrustY, Z: thrustZ}
	
	// Simplified propulsion moments (avoid complex calculations during RK4)
	propulsionMoments := Vector3{X: 0.0, Y: 0.0, Z: 0.0}
	
	// Approximate aircraft mass
	approxMass := 11600.0 * 0.453592 // P-51D loaded weight in kg
	
	// Add propulsion effects to derivatives
	derivatives.VelocityDot.X += propulsionForces.X / approxMass
	derivatives.VelocityDot.Y += propulsionForces.Y / approxMass
	derivatives.VelocityDot.Z += propulsionForces.Z / approxMass
	
	derivatives.AngularRateDot.X += propulsionMoments.X
	derivatives.AngularRateDot.Y += propulsionMoments.Y
	derivatives.AngularRateDot.Z += propulsionMoments.Z
	
	return derivatives, nil
}

// updatePropulsionSystem updates the propulsion system based on pilot inputs
func (engine *FlightDynamicsEngineWithPropulsion) updatePropulsionSystem(state *AircraftState, dt float64) {
	// Update propulsion system with throttle input
	throttleInput := state.Controls.Throttle
	
	// Apply mixture and propeller controls if available
	if engine.UseRealisticPropulsion {
		// In realistic mode, mixture and prop pitch affect engine performance
		// For now, keep it simple but extensible
		engine.Propulsion.Update(throttleInput, dt)
	} else {
		// Simple mode: just throttle
		engine.Propulsion.Update(throttleInput, dt)
	}
}

// updateAircraftWeight updates aircraft mass based on fuel consumption
func (engine *FlightDynamicsEngineWithPropulsion) updateAircraftWeight(state *AircraftState) {
	currentFuelWeight := engine.Propulsion.FuelSystem.TotalContents
	fuelBurned := engine.LastFuelWeight - currentFuelWeight
	
	if fuelBurned > 0 {
		// For now, we'll track fuel consumption but not modify aircraft mass
		// This would be handled by a mass/balance system in a complete implementation
		engine.LastFuelWeight = currentFuelWeight
	}
}

// updatePropulsionProperties updates the property manager with propulsion data
func (engine *FlightDynamicsEngineWithPropulsion) updatePropulsionProperties(state *AircraftState) {
	// Update propulsion properties in the FCS property manager
	engine.Propulsion.UpdateProperties(engine.FCS.Properties)
	
	// Add additional derived properties that aerodynamics might need
	props := engine.FCS.Properties
	
	// Propeller-induced dynamic pressure for aerodynamic effects
	propInducedVel := props.Get("propulsion/engine/prop-induced-velocity_fps")
	if propInducedVel > 0 {
		// Calculate thrust-enhanced dynamic pressure (from JSBSim XML line 1325)
		rho := state.Density // Use atmospheric density from state
		qbar := 0.5 * rho * (state.Velocity.X*state.Velocity.X + state.Velocity.Y*state.Velocity.Y + state.Velocity.Z*state.Velocity.Z)
		
		// Add propeller slipstream effect (simplified)
		propQbar := 0.5 * rho * (propInducedVel*0.3048)*(propInducedVel*0.3048) // Convert fps to m/s
		thrustQbar := qbar + propQbar
		
		props.Set("aero/thrust-qbar_psf", thrustQbar/47.88) // Convert to psf for JSBSim compatibility
	}
	
	// Engine power loading effects
	thrustLbs := props.Get("propulsion/engine/thrust-lbs")
	if thrustLbs > 0 {
		// Use approximate aircraft weight (would be better to have actual mass in state)
		approxWeight := 11600.0 * 4.448222 // P-51D loaded weight in Newtons
		powerLoading := approxWeight / (thrustLbs * 4.448222) // Weight/Thrust ratio
		props.Set("propulsion/power-loading", powerLoading)
	}
}



// calculatePropulsionForces calculates propulsion forces in body frame
func (engine *FlightDynamicsEngineWithPropulsion) calculatePropulsionForces(state *AircraftState) Vector3 {
	// Get thrust in Newtons
	thrust := engine.Propulsion.GetThrust()
	
	// Apply thrust along aircraft X-axis (forward) accounting for engine orientation
	// Engine orientation from XML: Roll: -4.0°, Pitch: 2.5°, Yaw: 0°
	engineRoll := engine.Propulsion.Orientation.X   // -4.0°
	enginePitch := engine.Propulsion.Orientation.Y  // 2.5°
	engineYaw := engine.Propulsion.Orientation.Z    // 0°
	
	// Transform thrust vector by engine orientation
	// Simplified: assume thrust is primarily along X-axis with small pitch/roll components
	thrustX := thrust * math.Cos(enginePitch) * math.Cos(engineYaw)
	thrustY := thrust * math.Sin(engineRoll) * math.Cos(enginePitch) // Small roll component
	thrustZ := thrust * math.Sin(enginePitch)                        // Small pitch component
	
	return Vector3{
		X: thrustX,  // Forward thrust (primary)
		Y: thrustY,  // Side thrust (from engine roll)
		Z: thrustZ,  // Vertical thrust (from engine pitch)
	}
}

// calculatePropulsionMoments calculates propulsion moments about CG
func (engine *FlightDynamicsEngineWithPropulsion) calculatePropulsionMoments(state *AircraftState) Vector3 {
	// Get engine torque
	torque := engine.Propulsion.GetTorque()
	
	// Engine position relative to CG (from XML: 36" = 0.9144m forward of datum)
	enginePos := engine.Propulsion.Position
	
	// Calculate moment arm from engine to CG
	// Assume CG is at aircraft datum for simplicity
	momentArm := enginePos
	
	// Thrust forces
	thrustForces := engine.calculatePropulsionForces(state)
	
	// Moments from thrust offset from CG
	thrustMoments := momentArm.Cross(thrustForces)
	
	// Engine torque reaction (opposite to propeller rotation)
	// Propeller rotates clockwise when viewed from behind (right-hand rule)
	engineTorque := Vector3{
		X: -torque, // Roll moment from engine torque reaction
		Y: 0.0,     // No pitch moment from torque
		Z: 0.0,     // No yaw moment from torque
	}
	
	// P-factor and spiraling slipstream effects (simplified)
	// These would be more complex in reality, involving angle of attack and airspeed
	pfactor := engine.calculatePFactorMoments(state)
	
	return Vector3{
		X: thrustMoments.X + engineTorque.X + pfactor.X,
		Y: thrustMoments.Y + engineTorque.Y + pfactor.Y,
		Z: thrustMoments.Z + engineTorque.Z + pfactor.Z,
	}
}

// calculatePFactorMoments calculates P-factor and spiraling slipstream moments
func (engine *FlightDynamicsEngineWithPropulsion) calculatePFactorMoments(state *AircraftState) Vector3 {
	// P-factor: asymmetric propeller loading at high angles of attack
	alpha := math.Atan2(state.Velocity.Z, state.Velocity.X) // Angle of attack
	thrust := engine.Propulsion.GetThrust()
	
	// P-factor creates yaw moment proportional to thrust and angle of attack
	pFactorYaw := thrust * math.Sin(alpha) * 0.1 // Scaling factor
	
	// Spiraling slipstream creates roll moment
	propRPM := engine.Propulsion.Propeller.RPM
	slipstreamRoll := propRPM * thrust * 0.00001 // Very small effect
	
	return Vector3{
		X: slipstreamRoll, // Roll from spiraling slipstream
		Y: 0.0,            // No direct pitch effect
		Z: pFactorYaw,     // Yaw from P-factor
	}
}

// applyConstraints applies physical constraints to the new state
func (engine *FlightDynamicsEngineWithPropulsion) applyConstraints(state *AircraftState) {
	// Additional propulsion-related constraints
	
	// Minimum fuel weight
	if engine.Propulsion.FuelSystem.TotalContents < 0 {
		engine.Propulsion.FuelSystem.TotalContents = 0
	}
	
	// Engine RPM limits
	if engine.Propulsion.Engine.RPM > engine.Propulsion.Engine.MaxRPM {
		engine.Propulsion.Engine.RPM = engine.Propulsion.Engine.MaxRPM
	}
	if engine.Propulsion.Engine.RPM < 0 {
		engine.Propulsion.Engine.RPM = 0
	}
}

// validateState validates the integrated state for NaN/Inf values
func (engine *FlightDynamicsEngineWithPropulsion) validateState(state *AircraftState) {
	// Additional propulsion validation
	if math.IsNaN(engine.Propulsion.Propeller.Thrust) || math.IsInf(engine.Propulsion.Propeller.Thrust, 0) {
		engine.Propulsion.Propeller.Thrust = 0.0
	}
	
	if math.IsNaN(engine.Propulsion.Engine.RPM) || math.IsInf(engine.Propulsion.Engine.RPM, 0) {
		engine.Propulsion.Engine.RPM = 0.0
	}
}

// GetPropulsionStatus returns current propulsion system status
func (engine *FlightDynamicsEngineWithPropulsion) GetPropulsionStatus() string {
	return engine.Propulsion.String()
}

// SetThrottle sets the throttle input directly
func (engine *FlightDynamicsEngineWithPropulsion) SetThrottle(throttle float64) {
	// Clamp throttle to valid range
	throttle = math.Max(0.0, math.Min(1.0, throttle))
	
	// This will be applied in the next simulation step
	// For immediate effect, we could update the propulsion system directly
	engine.Propulsion.Update(throttle, 0.01)
}

// GetFuelRemaining returns remaining fuel in lbs and percentage
func (engine *FlightDynamicsEngineWithPropulsion) GetFuelRemaining() (float64, float64) {
	remaining := engine.Propulsion.FuelSystem.TotalContents
	capacity := engine.Propulsion.FuelSystem.TotalCapacity
	percentage := remaining / capacity * 100.0
	return remaining, percentage
}

// GetThrustToWeightRatio returns current thrust-to-weight ratio
func (engine *FlightDynamicsEngineWithPropulsion) GetThrustToWeightRatio() float64 {
	thrust := engine.Propulsion.GetThrust()
	weight := engine.LastFuelWeight * 4.448222 // Convert lbs to Newtons
	if weight > 0 {
		return thrust / weight
	}
	return 0.0
}
