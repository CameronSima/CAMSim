package main

import (
	"fmt"
	"math"
)

// ForcesMomentsCalculator computes forces and moments acting on the aircraft
type ForcesMomentsCalculator struct {
	Config       *JSBSimConfig
	Mass         float64  // Aircraft mass in kg
	Inertia      Matrix3  // Moment of inertia tensor
	CG           Vector3  // Center of gravity position
	Reference    ReferenceData // Reference dimensions
}

// Matrix3 represents a 3x3 matrix for inertia tensor
type Matrix3 struct {
	XX, XY, XZ float64
	YX, YY, YZ float64
	ZX, ZY, ZZ float64
}

// ReferenceData contains aircraft reference dimensions
type ReferenceData struct {
	WingArea   float64 // Wing area in m²
	WingSpan   float64 // Wing span in m
	Chord      float64 // Mean aerodynamic chord in m
	EmptyMass  float64 // Empty mass in kg
}

// ForceMomentComponents represents the complete force and moment breakdown
type ForceMomentComponents struct {
	// Forces in body frame (N)
	Aerodynamic struct {
		Lift  float64 // Z-axis (negative for lift in NED)
		Drag  float64 // X-axis (negative for drag)
		Side  float64 // Y-axis
	}
	
	Propulsion struct {
		Thrust float64 // X-axis (positive forward)
		Torque float64 // Propeller torque about X-axis
	}
	
	Gravity struct {
		Weight Vector3 // Gravitational force in body frame
	}
	
	// Moments about body axes (N·m)
	Moments struct {
		Roll  float64 // L - moment about X-axis
		Pitch float64 // M - moment about Y-axis  
		Yaw   float64 // N - moment about Z-axis
	}
	
	// Totals
	TotalForce  Vector3 // Sum of all forces
	TotalMoment Vector3 // Sum of all moments
}

// NewForcesMomentsCalculator creates a new calculator from JSBSim config
func NewForcesMomentsCalculator(config *JSBSimConfig) *ForcesMomentsCalculator {
	calc := &ForcesMomentsCalculator{
		Config: config,
	}
	
	// Extract reference data from config
	if config.Metrics != nil {
		if config.Metrics.WingArea != nil {
			calc.Reference.WingArea = config.Metrics.WingArea.Value
		}
		if config.Metrics.WingSpan != nil {
			calc.Reference.WingSpan = config.Metrics.WingSpan.Value
		}
		if config.Metrics.Chord != nil {
			calc.Reference.Chord = config.Metrics.Chord.Value
		}
	}
	
	if config.MassBalance != nil {
		if config.MassBalance.EmptyMass != nil {
			calc.Reference.EmptyMass = config.MassBalance.EmptyMass.Value
			calc.Mass = config.MassBalance.EmptyMass.Value
		}
		
		// Set up inertia tensor (simplified for now)
		calc.Inertia = Matrix3{
			XX: calc.Mass * calc.Reference.WingSpan * calc.Reference.WingSpan / 12.0, // Roll inertia
			YY: calc.Mass * calc.Reference.Chord * calc.Reference.Chord / 12.0,     // Pitch inertia
			ZZ: calc.Mass * (calc.Reference.WingSpan*calc.Reference.WingSpan + calc.Reference.Chord*calc.Reference.Chord) / 12.0, // Yaw inertia
			XY: 0, XZ: 0, YZ: 0, YX: 0, ZX: 0, ZY: 0, // Assume principal axes
		}
	}
	
	return calc
}

// CalculateForcesMoments computes all forces and moments acting on the aircraft
func (calc *ForcesMomentsCalculator) CalculateForcesMoments(state *AircraftState) (*ForceMomentComponents, error) {
	components := &ForceMomentComponents{}
	
	// Get property map for function evaluation
	properties := state.ToPropertyMap()
	
	// Calculate aerodynamic forces
	err := calc.calculateAerodynamicForces(state, properties, components)
	if err != nil {
		return nil, fmt.Errorf("aerodynamic forces calculation failed: %v", err)
	}
	
	// Calculate propulsive forces
	calc.calculatePropulsiveForces(state, properties, components)
	
	// Calculate gravitational forces
	calc.calculateGravitationalForces(state, components)
	
	// Calculate moments
	err = calc.calculateMoments(state, properties, components)
	if err != nil {
		return nil, fmt.Errorf("moments calculation failed: %v", err)
	}
	
	// Sum total forces and moments
	calc.sumTotalForcesMoments(components)
	
	return components, nil
}

// calculateAerodynamicForces computes lift, drag, and side forces
func (calc *ForcesMomentsCalculator) calculateAerodynamicForces(state *AircraftState, properties map[string]float64, components *ForceMomentComponents) error {
	if calc.Config.Aerodynamics == nil {
		return fmt.Errorf("no aerodynamics configuration")
	}
	
	// Initialize force totals (JSBSim functions return forces in pounds, not coefficients)
	var liftForce, dragForce, sideForce float64
	
	// Evaluate aerodynamic force functions
	for _, axis := range calc.Config.Aerodynamics.Axis {
		switch axis.Name {
		case "LIFT":
			for _, function := range axis.Function {
				force, err := EvaluateFunction(function, properties)
				if err == nil {
					liftForce += force // Force in pounds
				}
			}
			
		case "DRAG":
			for _, function := range axis.Function {
				force, err := EvaluateFunction(function, properties)
				if err == nil {
					dragForce += force // Force in pounds
				}
			}
			
		case "SIDE":
			for _, function := range axis.Function {
				force, err := EvaluateFunction(function, properties)
				if err == nil {
					sideForce += force // Force in pounds
				}
			}
		}
	}
	
	// Convert forces from pounds to Newtons and apply to body frame
	const LB_TO_N = 4.44822
	components.Aerodynamic.Lift = -liftForce * LB_TO_N  // Negative Z in NED for positive lift
	components.Aerodynamic.Drag = -dragForce * LB_TO_N  // Negative X for drag opposing motion
	components.Aerodynamic.Side = sideForce * LB_TO_N   // Positive Y for right side force
	
	return nil
}

// calculatePropulsiveForces computes engine thrust and propeller effects
func (calc *ForcesMomentsCalculator) calculatePropulsiveForces(state *AircraftState, properties map[string]float64, components *ForceMomentComponents) {
	// Simplified thrust model based on throttle setting
	throttle := state.Controls.Throttle
	
	// Maximum thrust (simplified - should be from engine tables)
	maxThrust := 8000.0 // Newtons (approximate for P-51D)
	
	// Thrust varies with throttle and atmospheric density
	densityRatio := state.Density / 1.225 // Ratio to sea level density
	thrustAvailable := maxThrust * densityRatio
	
	// Current thrust based on throttle
	components.Propulsion.Thrust = throttle * thrustAvailable
	
	// Propeller torque (simplified)
	// Torque = Power / Angular_velocity, approximated
	power := components.Propulsion.Thrust * state.TrueAirspeed / 0.8 // Propeller efficiency ~80%
	propRPM := 2700.0 // Typical P-51D prop RPM
	propOmega := propRPM * 2.0 * math.Pi / 60.0 // rad/s
	
	if propOmega > 0 {
		components.Propulsion.Torque = power / propOmega
	}
}

// calculateGravitationalForces computes weight in body frame
func (calc *ForcesMomentsCalculator) calculateGravitationalForces(state *AircraftState, components *ForceMomentComponents) {
	// Weight always points down in Earth frame
	weightEarth := Vector3{X: 0, Y: 0, Z: calc.Mass * 9.81}
	
	// Transform to body frame
	// Need inverse rotation: body = q^-1 * earth * q
	qInv := Quaternion{W: state.Orientation.W, X: -state.Orientation.X, Y: -state.Orientation.Y, Z: -state.Orientation.Z}
	components.Gravity.Weight = qInv.RotateVector(weightEarth)
}

// calculateMoments computes roll, pitch, and yaw moments
func (calc *ForcesMomentsCalculator) calculateMoments(state *AircraftState, properties map[string]float64, components *ForceMomentComponents) error {
	if calc.Config.Aerodynamics == nil {
		return fmt.Errorf("no aerodynamics configuration")
	}
	
	// Dynamic pressure and reference dimensions
	qS := state.DynamicPressure * calc.Reference.WingArea
	qSb := qS * calc.Reference.WingSpan  // For roll moment
	qSc := qS * calc.Reference.Chord     // For pitch moment
	
	// Initialize moment coefficients
	var Cl, Cm, Cn float64
	
	// Evaluate moment coefficient functions
	for _, axis := range calc.Config.Aerodynamics.Axis {
		switch axis.Name {
		case "ROLL":
			for _, function := range axis.Function {
				coeff, err := EvaluateFunction(function, properties)
				if err == nil {
					Cl += coeff
				}
			}
			
		case "PITCH":
			for _, function := range axis.Function {
				coeff, err := EvaluateFunction(function, properties)
				if err == nil {
					Cm += coeff
				}
			}
			
		case "YAW":
			for _, function := range axis.Function {
				coeff, err := EvaluateFunction(function, properties)
				if err == nil {
					Cn += coeff
				}
			}
		}
	}
	
	// Convert coefficients to moments
	components.Moments.Roll = Cl * qSb
	components.Moments.Pitch = Cm * qSc
	components.Moments.Yaw = Cn * qSb
	
	// Add propeller torque to roll moment
	components.Moments.Roll += components.Propulsion.Torque
	
	return nil
}

// sumTotalForcesMoments computes the total forces and moments
func (calc *ForcesMomentsCalculator) sumTotalForcesMoments(components *ForceMomentComponents) {
	// Sum forces in body frame
	components.TotalForce = Vector3{
		X: components.Aerodynamic.Drag + components.Propulsion.Thrust + components.Gravity.Weight.X,
		Y: components.Aerodynamic.Side + components.Gravity.Weight.Y,
		Z: components.Aerodynamic.Lift + components.Gravity.Weight.Z,
	}
	
	// Sum moments
	components.TotalMoment = Vector3{
		X: components.Moments.Roll,
		Y: components.Moments.Pitch,
		Z: components.Moments.Yaw,
	}
}

// CalculateStateDerivatives computes the time derivatives of the aircraft state
func (calc *ForcesMomentsCalculator) CalculateStateDerivatives(state *AircraftState, components *ForceMomentComponents) *StateDerivatives {
	derivatives := &StateDerivatives{}
	
	// Linear acceleration (F = ma)
	derivatives.VelocityDot = components.TotalForce.Scale(1.0 / calc.Mass)
	
	// Angular acceleration (M = I * α)
	derivatives.AngularRateDot = calc.calculateAngularAcceleration(state, components.TotalMoment)
	
	// Altitude rate (climb/descent)
	earthVel := state.Orientation.RotateVector(state.Velocity)
	derivatives.AltitudeDot = -earthVel.Z // Negative Z is climb in NED
	
	// Mass rate (fuel consumption - simplified)
	fuelFlow := calc.estimateFuelFlow(components.Propulsion.Thrust)
	derivatives.MassDot = -fuelFlow
	
	return derivatives
}

// calculateAngularAcceleration computes angular acceleration from moments
func (calc *ForcesMomentsCalculator) calculateAngularAcceleration(state *AircraftState, moments Vector3) Vector3 {
	// Simplified approach: assume principal axes, ignore gyroscopic effects for now
	// α = I^-1 * M
	
	angularAccel := Vector3{
		X: moments.X / calc.Inertia.XX, // Roll acceleration
		Y: moments.Y / calc.Inertia.YY, // Pitch acceleration
		Z: moments.Z / calc.Inertia.ZZ, // Yaw acceleration
	}
	
	return angularAccel
}

// estimateFuelFlow provides a simplified fuel consumption model
func (calc *ForcesMomentsCalculator) estimateFuelFlow(thrust float64) float64 {
	// Simplified: fuel flow proportional to thrust
	// Typical P-51D: ~300 gal/hr at max power, ~1134 kg/hr
	maxFuelFlow := 1134.0 / 3600.0 // kg/s at max thrust
	maxThrust := 8000.0            // N
	
	if maxThrust > 0 {
		return (thrust / maxThrust) * maxFuelFlow
	}
	return 0
}

// FlightDynamicsEngine combines forces/moments calculator with integration
type FlightDynamicsEngine struct {
	Calculator *ForcesMomentsCalculator
	Integrator Integrator
	Statistics *FlightStatistics
}

// FlightStatistics tracks flight performance metrics
type FlightStatistics struct {
	MaxLoadFactor    float64
	MaxClimbRate     float64
	MaxSpeed         float64
	MaxAltitude      float64
	TotalFuelBurned  float64
	FlightTime       float64
}

// NewFlightDynamicsEngine creates a complete flight dynamics simulation engine
func NewFlightDynamicsEngine(config *JSBSimConfig, integrator Integrator) *FlightDynamicsEngine {
	return &FlightDynamicsEngine{
		Calculator: NewForcesMomentsCalculator(config),
		Integrator: integrator,
		Statistics: &FlightStatistics{},
	}
}

// Step advances the simulation by one time step
func (fde *FlightDynamicsEngine) Step(state *AircraftState, dt float64) (*AircraftState, error) {
	// Calculate forces and moments
	components, err := fde.Calculator.CalculateForcesMoments(state)
	if err != nil {
		return nil, err
	}
	
	// Calculate state derivatives
	derivatives := fde.Calculator.CalculateStateDerivatives(state, components)
	
	// Integrate to new state
	newState := fde.Integrator.Integrate(state, derivatives, dt)
	
	// Update flight statistics
	fde.updateStatistics(newState, components, dt)
	
	// Store forces and moments in state for analysis
	newState.Forces.Total = components.TotalForce
	newState.Moments.Total = components.TotalMoment
	newState.Forces.Aerodynamic = Vector3{
		X: components.Aerodynamic.Drag,
		Y: components.Aerodynamic.Side,
		Z: components.Aerodynamic.Lift,
	}
	newState.Forces.Propulsive = Vector3{X: components.Propulsion.Thrust, Y: 0, Z: 0}
	newState.Forces.Gravity = components.Gravity.Weight
	
	return newState, nil
}

// updateStatistics tracks flight performance metrics
func (fde *FlightDynamicsEngine) updateStatistics(state *AircraftState, components *ForceMomentComponents, dt float64) {
	// Load factor (g-force)
	totalAccel := components.TotalForce.Magnitude() / fde.Calculator.Mass
	loadFactor := totalAccel / 9.81
	if loadFactor > fde.Statistics.MaxLoadFactor {
		fde.Statistics.MaxLoadFactor = loadFactor
	}
	
	// Climb rate
	earthVel := state.Orientation.RotateVector(state.Velocity)
	climbRate := -earthVel.Z // Negative Z is climb
	if math.Abs(climbRate) > fde.Statistics.MaxClimbRate {
		fde.Statistics.MaxClimbRate = math.Abs(climbRate)
	}
	
	// Speed and altitude
	if state.TrueAirspeed > fde.Statistics.MaxSpeed {
		fde.Statistics.MaxSpeed = state.TrueAirspeed
	}
	if state.Altitude > fde.Statistics.MaxAltitude {
		fde.Statistics.MaxAltitude = state.Altitude
	}
	
	// Fuel consumption
	fuelFlow := fde.Calculator.estimateFuelFlow(components.Propulsion.Thrust)
	fde.Statistics.TotalFuelBurned += fuelFlow * dt
	fde.Statistics.FlightTime += dt
}

// GetPerformanceReport generates a performance summary
func (fde *FlightDynamicsEngine) GetPerformanceReport() string {
	stats := fde.Statistics
	return fmt.Sprintf(
		"Flight Performance Report:\n"+
		"  Flight Time: %.1f seconds\n"+
		"  Max Load Factor: %.2f g\n"+
		"  Max Climb Rate: %.1f m/s (%.0f ft/min)\n"+
		"  Max Speed: %.1f m/s (%.1f kt)\n"+
		"  Max Altitude: %.0f m (%.0f ft)\n"+
		"  Total Fuel Burned: %.2f kg\n"+
		"  Average Fuel Flow: %.3f kg/s",
		stats.FlightTime,
		stats.MaxLoadFactor,
		stats.MaxClimbRate, stats.MaxClimbRate*60*M_TO_FT,
		stats.MaxSpeed, stats.MaxSpeed*MS_TO_KT,
		stats.MaxAltitude, stats.MaxAltitude*M_TO_FT,
		stats.TotalFuelBurned,
		stats.TotalFuelBurned/math.Max(stats.FlightTime, 1.0),
	)
}

// TrimCalculator finds equilibrium control settings for steady flight
type TrimCalculator struct {
	Engine *FlightDynamicsEngine
}

// NewTrimCalculator creates a trim calculator
func NewTrimCalculator(engine *FlightDynamicsEngine) *TrimCalculator {
	return &TrimCalculator{Engine: engine}
}

// FindTrim finds control inputs for steady level flight at given conditions
func (tc *TrimCalculator) FindTrim(targetSpeed, targetAltitude float64) (ControlInputs, error) {
	// Simplified trim calculation using iterative approach
	controls := NewControlInputs()
	
	// Set initial throttle based on desired speed
	controls.Throttle = math.Min(targetSpeed/150.0, 1.0) // Rough estimate
	
	// Adjust elevator for level flight (simplified)
	// In reality, this would require iterative solving
	controls.Elevator = 0.0 // Start neutral
	
	// This is a placeholder - real trim would require:
	// 1. Newton-Raphson iteration
	// 2. Multiple control variables optimization
	// 3. Constraint satisfaction for forces/moments balance
	
	return controls, nil
}

// AerodynamicAnalysis provides detailed aerodynamic performance analysis
type AerodynamicAnalysis struct {
	AlphaRange []float64 // Angle of attack sweep
	CLCurve    []float64 // Lift coefficients
	CDCurve    []float64 // Drag coefficients
	LDRatio    []float64 // Lift-to-drag ratios
	MaxLD      float64   // Maximum L/D ratio
	BestAlpha  float64   // Alpha for best L/D
	StallAlpha float64   // Stall angle of attack
}

// PerformAerodynamicAnalysis conducts a comprehensive aero analysis
func (calc *ForcesMomentsCalculator) PerformAerodynamicAnalysis(baseState *AircraftState) *AerodynamicAnalysis {
	analysis := &AerodynamicAnalysis{}
	
	// Define alpha sweep from -10° to +20°
	alphaStart := -10.0 * DEG_TO_RAD
	alphaEnd := 20.0 * DEG_TO_RAD
	steps := 31
	
	analysis.AlphaRange = make([]float64, steps)
	analysis.CLCurve = make([]float64, steps)
	analysis.CDCurve = make([]float64, steps)
	analysis.LDRatio = make([]float64, steps)
	
	for i := 0; i < steps; i++ {
		alpha := alphaStart + float64(i)*(alphaEnd-alphaStart)/float64(steps-1)
		analysis.AlphaRange[i] = alpha
		
		// Create state at this alpha by adjusting velocity vector
		testState := baseState.Copy()
		
		// Calculate velocity components for desired alpha
		// alpha = atan(w / u) where w is vertical velocity (positive up)
		speed := testState.Velocity.Magnitude()
		u := speed * math.Cos(alpha)  // Forward velocity
		w := speed * math.Sin(alpha)  // Vertical velocity (positive up)
		
		testState.Velocity = Vector3{X: u, Y: testState.Velocity.Y, Z: -w} // NED: Z down
		testState.UpdateDerivedParameters()
		
		// Calculate forces
		properties := testState.ToPropertyMap()
		components := &ForceMomentComponents{}
		calc.calculateAerodynamicForces(testState, properties, components)
		
		// Extract coefficients
		qS := testState.DynamicPressure * calc.Reference.WingArea
		if qS > 0 {
			CL := -components.Aerodynamic.Lift / qS
			CD := -components.Aerodynamic.Drag / qS
			
			analysis.CLCurve[i] = CL
			analysis.CDCurve[i] = CD
			
			if CD > 0 {
				LD := CL / CD
				analysis.LDRatio[i] = LD
				
				if LD > analysis.MaxLD {
					analysis.MaxLD = LD
					analysis.BestAlpha = alpha
				}
			}
		}
	}
	
	// Find stall alpha (where CL starts decreasing significantly)
	maxCL := 0.0
	for i, CL := range analysis.CLCurve {
		if CL > maxCL {
			maxCL = CL
			analysis.StallAlpha = analysis.AlphaRange[i]
		}
	}
	
	return analysis
}

// PerformanceEnvelope calculates aircraft performance across flight envelope
type PerformanceEnvelope struct {
	Altitudes    []float64   // Test altitudes
	MaxSpeeds    []float64   // Maximum speeds at each altitude
	ClimbRates   []float64   // Climb rates at each altitude
	ServiceCeiling float64   // Maximum operational altitude
	AbsoluteCeiling float64  // Theoretical maximum altitude
}

// CalculatePerformanceEnvelope determines aircraft performance limits
func (fde *FlightDynamicsEngine) CalculatePerformanceEnvelope() *PerformanceEnvelope {
	envelope := &PerformanceEnvelope{}
	
	// Test altitudes from sea level to 15,000m
	altitudes := []float64{0, 1000, 2000, 3000, 5000, 7000, 10000, 12000, 15000}
	envelope.Altitudes = altitudes
	envelope.MaxSpeeds = make([]float64, len(altitudes))
	envelope.ClimbRates = make([]float64, len(altitudes))
	
	for i, altitude := range altitudes {
		// Create test state at this altitude
		testState := NewAircraftState()
		testState.Altitude = altitude
		testState.Velocity = Vector3{X: 100.0, Y: 0, Z: 0} // Start at 100 m/s
		testState.UpdateAtmosphere()
		testState.UpdateDerivedParameters()
		
		// Full throttle performance
		testState.Controls.Throttle = 1.0
		
		// Calculate forces at this condition
		components, err := fde.Calculator.CalculateForcesMoments(testState)
		if err == nil {
			// Maximum speed where thrust = drag
			// Simplified: assume max speed reached when thrust available
			envelope.MaxSpeeds[i] = testState.TrueAirspeed + components.Propulsion.Thrust/100.0
			
			// Climb rate from excess thrust
			excessThrust := components.Propulsion.Thrust + components.Aerodynamic.Drag
			if excessThrust > 0 {
				envelope.ClimbRates[i] = excessThrust / fde.Calculator.Mass
			}
		}
	}
	
	// Find service ceiling (where climb rate drops to 0.5 m/s)
	for i, climbRate := range envelope.ClimbRates {
		if climbRate < 0.5 && envelope.ServiceCeiling == 0 {
			envelope.ServiceCeiling = envelope.Altitudes[i]
		}
		if climbRate <= 0 && envelope.AbsoluteCeiling == 0 {
			envelope.AbsoluteCeiling = envelope.Altitudes[i]
		}
	}
	
	return envelope
}
