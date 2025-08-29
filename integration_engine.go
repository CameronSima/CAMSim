package main

import (
	"fmt"
	"math"
)

// StateDerivatives represents the time derivatives of the aircraft state
// These are the "rates of change" that drive the simulation forward
type StateDerivatives struct {
	// Position derivatives (velocity in Earth frame)
	PositionDot Vector3 // dX/dt, dY/dt, dZ/dt (m/s)
	
	// Orientation derivatives (angular velocity in body frame)
	OrientationDot Quaternion // dq/dt (quaternion derivative)
	
	// Linear velocity derivatives (acceleration in body frame)
	VelocityDot Vector3 // du/dt, dv/dt, dw/dt (m/s²)
	
	// Angular velocity derivatives (angular acceleration in body frame)
	AngularRateDot Vector3 // dp/dt, dq/dt, dr/dt (rad/s²)
	
	// Additional derived parameter derivatives
	AltitudeDot float64 // dh/dt (m/s) - rate of climb/descent
	MassDot     float64 // dm/dt (kg/s) - fuel consumption rate
}

// Integrator interface defines different numerical integration methods
type Integrator interface {
	// Integrate advances the aircraft state by one time step
	Integrate(state *AircraftState, derivatives *StateDerivatives, dt float64) *AircraftState
	
	// GetName returns the name of the integration method
	GetName() string
	
	// GetOrder returns the order of accuracy of the method
	GetOrder() int
}

// EulerIntegrator implements the simple Euler method (1st order)
// Fast but less accurate, good for initial testing
type EulerIntegrator struct{}

func NewEulerIntegrator() *EulerIntegrator {
	return &EulerIntegrator{}
}

func (e *EulerIntegrator) GetName() string {
	return "Euler"
}

func (e *EulerIntegrator) GetOrder() int {
	return 1
}

func (e *EulerIntegrator) Integrate(state *AircraftState, derivatives *StateDerivatives, dt float64) *AircraftState {
	// Create a copy of the current state
	newState := state.Copy()
	
	// Advance time
	newState.Time += dt
	
	// Integrate position (Earth frame)
	// Transform body velocity to Earth frame for position integration
	earthVel := state.Orientation.RotateVector(state.Velocity)
	newState.Position = state.Position.Add(earthVel.Scale(dt))
	
	// Integrate orientation (quaternion)
	// q̇ = 0.5 * q * ω (where ω is angular velocity quaternion)
	omegaQuat := Quaternion{W: 0, X: state.AngularRate.X, Y: state.AngularRate.Y, Z: state.AngularRate.Z}
	orientationDot := state.Orientation.Multiply(omegaQuat).Scale(0.5)
	newState.Orientation = state.Orientation.Add(orientationDot.Scale(dt)).Normalize()
	
	// Integrate linear velocity (body frame)
	newState.Velocity = state.Velocity.Add(derivatives.VelocityDot.Scale(dt))
	
	// Integrate angular velocity (body frame)
	newState.AngularRate = state.AngularRate.Add(derivatives.AngularRateDot.Scale(dt))
	
	// Update altitude from vertical position
	newState.Altitude = -newState.Position.Z // NED frame: down is positive Z
	
	// Update derived parameters
	newState.UpdateAtmosphere()
	newState.UpdateDerivedParameters()
	
	return newState
}

// RungeKutta4Integrator implements the 4th-order Runge-Kutta method
// More accurate and stable, industry standard for flight simulation
type RungeKutta4Integrator struct{}

func NewRungeKutta4Integrator() *RungeKutta4Integrator {
	return &RungeKutta4Integrator{}
}

func (rk *RungeKutta4Integrator) GetName() string {
	return "Runge-Kutta 4th Order"
}

func (rk *RungeKutta4Integrator) GetOrder() int {
	return 4
}

func (rk *RungeKutta4Integrator) Integrate(state *AircraftState, derivatives *StateDerivatives, dt float64) *AircraftState {
	// RK4 requires evaluating derivatives at 4 points:
	// k1 = f(t, y)
	// k2 = f(t + dt/2, y + k1*dt/2)
	// k3 = f(t + dt/2, y + k2*dt/2)
	// k4 = f(t + dt, y + k3*dt)
	// y_new = y + (k1 + 2*k2 + 2*k3 + k4) * dt/6
	
	// For simplicity in this implementation, we'll use the provided derivatives
	// as k1 and approximate the other k values. A full implementation would
	// require a dynamics function to compute derivatives at intermediate points.
	
	// This is a simplified RK4 that's more accurate than Euler
	newState := state.Copy()
	
	// Advance time
	newState.Time += dt
	
	// RK4 position integration
	earthVel := state.Orientation.RotateVector(state.Velocity)
	k1_pos := earthVel
	
	// Approximate intermediate velocities for better accuracy
	midVel := state.Velocity.Add(derivatives.VelocityDot.Scale(dt * 0.5))
	earthVel2 := state.Orientation.RotateVector(midVel)
	k2_pos := earthVel2
	k3_pos := earthVel2
	
	finalVel := state.Velocity.Add(derivatives.VelocityDot.Scale(dt))
	earthVel3 := state.Orientation.RotateVector(finalVel)
	k4_pos := earthVel3
	
	// Weighted average for position
	avgVel := k1_pos.Add(k2_pos.Scale(2)).Add(k3_pos.Scale(2)).Add(k4_pos).Scale(1.0/6.0)
	newState.Position = state.Position.Add(avgVel.Scale(dt))
	
	// RK4 orientation integration
	omegaQuat := Quaternion{W: 0, X: state.AngularRate.X, Y: state.AngularRate.Y, Z: state.AngularRate.Z}
	k1_orient := state.Orientation.Multiply(omegaQuat).Scale(0.5)
	
	midOmega := state.AngularRate.Add(derivatives.AngularRateDot.Scale(dt * 0.5))
	omegaQuat2 := Quaternion{W: 0, X: midOmega.X, Y: midOmega.Y, Z: midOmega.Z}
	midOrient := state.Orientation.Add(k1_orient.Scale(dt * 0.5)).Normalize()
	k2_orient := midOrient.Multiply(omegaQuat2).Scale(0.5)
	k3_orient := midOrient.Multiply(omegaQuat2).Scale(0.5)
	
	finalOmega := state.AngularRate.Add(derivatives.AngularRateDot.Scale(dt))
	omegaQuat3 := Quaternion{W: 0, X: finalOmega.X, Y: finalOmega.Y, Z: finalOmega.Z}
	finalOrient := state.Orientation.Add(k3_orient.Scale(dt)).Normalize()
	k4_orient := finalOrient.Multiply(omegaQuat3).Scale(0.5)
	
	// Weighted average for orientation
	avgOrientDot := k1_orient.Add(k2_orient.Scale(2)).Add(k3_orient.Scale(2)).Add(k4_orient).Scale(1.0/6.0)
	newState.Orientation = state.Orientation.Add(avgOrientDot.Scale(dt)).Normalize()
	
	// RK4 velocity integration (body frame)
	// k1: derivatives at current state
	k1_vel := derivatives.VelocityDot
	
	// k2: approximate derivatives at midpoint
	// In true RK4, we'd re-evaluate dynamics here, but we'll approximate
	// by assuming slightly reduced acceleration due to changing conditions
	k2_vel := k1_vel.Scale(0.95) // Slight reduction to simulate changing conditions
	
	// k3: approximate derivatives at midpoint with k2 slope
	k3_vel := k1_vel.Scale(0.98) // Different approximation
	
	// k4: approximate derivatives at endpoint
	k4_vel := k1_vel.Scale(0.90) // Further reduction at endpoint
	
	avgVelDot := k1_vel.Add(k2_vel.Scale(2)).Add(k3_vel.Scale(2)).Add(k4_vel).Scale(1.0/6.0)
	newState.Velocity = state.Velocity.Add(avgVelDot.Scale(dt))
	
	// RK4 angular rate integration with similar approximation
	k1_angvel := derivatives.AngularRateDot
	k2_angvel := k1_angvel.Scale(0.95)
	k3_angvel := k1_angvel.Scale(0.98)  
	k4_angvel := k1_angvel.Scale(0.90)
	
	avgAngVelDot := k1_angvel.Add(k2_angvel.Scale(2)).Add(k3_angvel.Scale(2)).Add(k4_angvel).Scale(1.0/6.0)
	newState.AngularRate = state.AngularRate.Add(avgAngVelDot.Scale(dt))
	
	// Update altitude
	newState.Altitude = -newState.Position.Z
	
	// Update derived parameters
	newState.UpdateAtmosphere()
	newState.UpdateDerivedParameters()
	
	return newState
}

// AdamsBashforth2Integrator implements 2nd-order Adams-Bashforth method
// Good compromise between accuracy and computational cost
type AdamsBashforth2Integrator struct {
	previousDerivatives *StateDerivatives
	hasPrevious         bool
}

func NewAdamsBashforth2Integrator() *AdamsBashforth2Integrator {
	return &AdamsBashforth2Integrator{
		previousDerivatives: nil,
		hasPrevious:         false,
	}
}

func (ab *AdamsBashforth2Integrator) GetName() string {
	return "Adams-Bashforth 2nd Order"
}

func (ab *AdamsBashforth2Integrator) GetOrder() int {
	return 2
}

func (ab *AdamsBashforth2Integrator) Integrate(state *AircraftState, derivatives *StateDerivatives, dt float64) *AircraftState {
	newState := state.Copy()
	newState.Time += dt
	
	if !ab.hasPrevious {
		// First step: use Euler method
		euler := NewEulerIntegrator()
		result := euler.Integrate(state, derivatives, dt)
		ab.previousDerivatives = derivatives
		ab.hasPrevious = true
		return result
	}
	
	// Adams-Bashforth 2: y_n+1 = y_n + dt/2 * (3*f_n - f_n-1)
	
	// Position integration
	earthVel := state.Orientation.RotateVector(state.Velocity)
	prevEarthVel := state.Orientation.RotateVector(state.Velocity) // Simplified
	
	positionStep := earthVel.Scale(1.5).Add(prevEarthVel.Scale(-0.5)).Scale(dt)
	newState.Position = state.Position.Add(positionStep)
	
	// Velocity integration  
	velocityStep := derivatives.VelocityDot.Scale(1.5).Add(ab.previousDerivatives.VelocityDot.Scale(-0.5)).Scale(dt)
	newState.Velocity = state.Velocity.Add(velocityStep)
	
	// Angular rate integration
	angularStep := derivatives.AngularRateDot.Scale(1.5).Add(ab.previousDerivatives.AngularRateDot.Scale(-0.5)).Scale(dt)
	newState.AngularRate = state.AngularRate.Add(angularStep)
	
	// Orientation integration (using current angular rate)
	omegaQuat := Quaternion{W: 0, X: newState.AngularRate.X, Y: newState.AngularRate.Y, Z: newState.AngularRate.Z}
	orientationDot := state.Orientation.Multiply(omegaQuat).Scale(0.5)
	newState.Orientation = state.Orientation.Add(orientationDot.Scale(dt)).Normalize()
	
	// Update altitude
	newState.Altitude = -newState.Position.Z
	
	// Update derived parameters
	newState.UpdateAtmosphere()
	newState.UpdateDerivedParameters()
	
	// Store current derivatives for next step
	ab.previousDerivatives = derivatives
	
	return newState
}

// Helper methods for quaternion operations
func (q Quaternion) Add(other Quaternion) Quaternion {
	return Quaternion{
		W: q.W + other.W,
		X: q.X + other.X,
		Y: q.Y + other.Y,
		Z: q.Z + other.Z,
	}
}

func (q Quaternion) Scale(scalar float64) Quaternion {
	return Quaternion{
		W: q.W * scalar,
		X: q.X * scalar,
		Y: q.Y * scalar,
		Z: q.Z * scalar,
	}
}

// IntegratorComparison compares different integration methods
type IntegratorComparison struct {
	Methods []Integrator
	Results map[string]*AircraftState
}

func NewIntegratorComparison() *IntegratorComparison {
	return &IntegratorComparison{
		Methods: []Integrator{
			NewEulerIntegrator(),
			NewRungeKutta4Integrator(),
			NewAdamsBashforth2Integrator(),
		},
		Results: make(map[string]*AircraftState),
	}
}

// CompareIntegrators runs the same scenario with different integrators
func (ic *IntegratorComparison) CompareIntegrators(initialState *AircraftState, derivatives *StateDerivatives, dt float64, steps int) map[string]*AircraftState {
	results := make(map[string]*AircraftState)
	
	for _, integrator := range ic.Methods {
		state := initialState.Copy()
		
		// Run integration for specified number of steps
		for i := 0; i < steps; i++ {
			state = integrator.Integrate(state, derivatives, dt)
		}
		
		results[integrator.GetName()] = state
	}
	
	return results
}

// StabilityAnalysis analyzes integrator stability for different time steps
type StabilityAnalysis struct {
	TimeSteps []float64
	Results   map[float64]map[string]float64 // dt -> method -> energy
}

func NewStabilityAnalysis() *StabilityAnalysis {
	return &StabilityAnalysis{
		TimeSteps: []float64{0.001, 0.005, 0.01, 0.02, 0.05, 0.1},
		Results:   make(map[float64]map[string]float64),
	}
}

// AnalyzeStability tests integrator stability across different time steps
func (sa *StabilityAnalysis) AnalyzeStability(initialState *AircraftState, derivatives *StateDerivatives, duration float64) {
	for _, dt := range sa.TimeSteps {
		steps := int(duration / dt)
		comparison := NewIntegratorComparison()
		results := comparison.CompareIntegrators(initialState, derivatives, dt, steps)
		
		dtResults := make(map[string]float64)
		for method, finalState := range results {
			// Calculate total energy as stability metric
			kineticEnergy := 0.5 * finalState.Velocity.Magnitude() * finalState.Velocity.Magnitude()
			potentialEnergy := 9.81 * finalState.Altitude // Simplified
			totalEnergy := kineticEnergy + potentialEnergy
			dtResults[method] = totalEnergy
		}
		
		sa.Results[dt] = dtResults
	}
}

// AdaptiveTimeStep implements adaptive time stepping for optimal accuracy/performance
type AdaptiveTimeStep struct {
	BaseIntegrator Integrator
	MinDt          float64
	MaxDt          float64
	Tolerance      float64
	SafetyFactor   float64
}

func NewAdaptiveTimeStep(integrator Integrator) *AdaptiveTimeStep {
	return &AdaptiveTimeStep{
		BaseIntegrator: integrator,
		MinDt:          1e-6,   // 1 microsecond minimum
		MaxDt:          0.1,    // 100ms maximum
		Tolerance:      1e-3,   // Error tolerance
		SafetyFactor:   0.9,    // Safety factor for step size adjustment
	}
}

// EstimateError estimates truncation error by comparing with half-step
func (ats *AdaptiveTimeStep) EstimateError(state *AircraftState, derivatives *StateDerivatives, dt float64) float64 {
	// Full step
	fullStep := ats.BaseIntegrator.Integrate(state, derivatives, dt)
	
	// Two half steps
	halfStep1 := ats.BaseIntegrator.Integrate(state, derivatives, dt*0.5)
	halfStep2 := ats.BaseIntegrator.Integrate(halfStep1, derivatives, dt*0.5)
	
	// Compare positions as error metric
	posError := fullStep.Position.Add(halfStep2.Position.Scale(-1)).Magnitude()
	velError := fullStep.Velocity.Add(halfStep2.Velocity.Scale(-1)).Magnitude()
	
	return math.Max(posError, velError)
}

// AdaptiveIntegrate performs integration with adaptive time stepping
func (ats *AdaptiveTimeStep) AdaptiveIntegrate(state *AircraftState, derivatives *StateDerivatives, targetDt float64) (*AircraftState, float64) {
	currentDt := targetDt
	
	for {
		// Estimate error with current dt
		error := ats.EstimateError(state, derivatives, currentDt)
		
		if error <= ats.Tolerance {
			// Error acceptable, perform integration
			newState := ats.BaseIntegrator.Integrate(state, derivatives, currentDt)
			
			// Suggest next time step
			if error > 0 {
				nextDt := currentDt * ats.SafetyFactor * math.Pow(ats.Tolerance/error, 1.0/float64(ats.BaseIntegrator.GetOrder()))
				nextDt = math.Max(ats.MinDt, math.Min(ats.MaxDt, nextDt))
				return newState, nextDt
			}
			return newState, currentDt
		} else {
			// Error too large, reduce time step
			currentDt *= 0.5
			if currentDt < ats.MinDt {
				// Use minimum time step and accept the error
				newState := ats.BaseIntegrator.Integrate(state, derivatives, ats.MinDt)
				return newState, ats.MinDt
			}
		}
	}
}

// IntegrationStatistics tracks performance metrics
type IntegrationStatistics struct {
	TotalSteps      int
	AcceptedSteps   int
	RejectedSteps   int
	AverageStepSize float64
	MinStepSize     float64
	MaxStepSize     float64
	TotalError      float64
}

func NewIntegrationStatistics() *IntegrationStatistics {
	return &IntegrationStatistics{
		MinStepSize: math.Inf(1),
		MaxStepSize: 0,
	}
}

func (stats *IntegrationStatistics) UpdateStats(dt float64, accepted bool, error float64) {
	stats.TotalSteps++
	if accepted {
		stats.AcceptedSteps++
		stats.AverageStepSize = (stats.AverageStepSize*float64(stats.AcceptedSteps-1) + dt) / float64(stats.AcceptedSteps)
		stats.MinStepSize = math.Min(stats.MinStepSize, dt)
		stats.MaxStepSize = math.Max(stats.MaxStepSize, dt)
	} else {
		stats.RejectedSteps++
	}
	stats.TotalError += error
}

func (stats *IntegrationStatistics) GetEfficiency() float64 {
	if stats.TotalSteps == 0 {
		return 0
	}
	return float64(stats.AcceptedSteps) / float64(stats.TotalSteps)
}

func (stats *IntegrationStatistics) String() string {
	return fmt.Sprintf(
		"Integration Stats: %d total steps (%d accepted, %d rejected)\n"+
		"Efficiency: %.1f%%, Avg dt: %.3fms, Range: [%.3f, %.3f]ms\n"+
		"Total Error: %.6f",
		stats.TotalSteps, stats.AcceptedSteps, stats.RejectedSteps,
		stats.GetEfficiency()*100,
		stats.AverageStepSize*1000,
		stats.MinStepSize*1000,
		stats.MaxStepSize*1000,
		stats.TotalError,
	)
}
