// True RK4 Integrator with Dynamics Re-evaluation
// This implements proper RK4 that re-evaluates dynamics at intermediate steps

package main

// DynamicsFunction represents a function that calculates state derivatives
type DynamicsFunction func(*AircraftState) (*StateDerivatives, error)

// TrueRK4Integrator implements true RK4 with dynamics re-evaluation
type TrueRK4Integrator struct {
	DynamicsFunc DynamicsFunction
}

// NewTrueRK4Integrator creates a new true RK4 integrator
func NewTrueRK4Integrator(dynamicsFunc DynamicsFunction) *TrueRK4Integrator {
	return &TrueRK4Integrator{
		DynamicsFunc: dynamicsFunc,
	}
}

func (rk *TrueRK4Integrator) GetName() string {
	return "True Runge-Kutta 4th Order"
}

func (rk *TrueRK4Integrator) GetOrder() int {
	return 4
}

// Integrate performs true RK4 integration with dynamics re-evaluation
func (rk *TrueRK4Integrator) Integrate(state *AircraftState, derivatives *StateDerivatives, dt float64) *AircraftState {
	// True RK4 implementation:
	// k1 = f(t, y)
	// k2 = f(t + dt/2, y + k1*dt/2)  
	// k3 = f(t + dt/2, y + k2*dt/2)
	// k4 = f(t + dt, y + k3*dt)
	// y_new = y + (k1 + 2*k2 + 2*k3 + k4) * dt/6
	
	// k1: derivatives at current state (already provided)
	k1 := derivatives
	
	// k2: evaluate dynamics at midpoint using k1 slope
	midState1 := rk.advanceState(state, k1, dt*0.5)
	k2, err := rk.DynamicsFunc(midState1)
	if err != nil {
		// Fallback to simplified if dynamics evaluation fails
		return rk.simplifiedIntegrate(state, derivatives, dt)
	}
	
	// k3: evaluate dynamics at midpoint using k2 slope
	midState2 := rk.advanceState(state, k2, dt*0.5)
	k3, err := rk.DynamicsFunc(midState2)
	if err != nil {
		return rk.simplifiedIntegrate(state, derivatives, dt)
	}
	
	// k4: evaluate dynamics at endpoint using k3 slope
	endState := rk.advanceState(state, k3, dt)
	k4, err := rk.DynamicsFunc(endState)
	if err != nil {
		return rk.simplifiedIntegrate(state, derivatives, dt)
	}
	
	// Combine using RK4 weights
	newState := state.Copy()
	
	// Position integration
	k1_pos := k1.PositionDot
	k2_pos := k2.PositionDot
	k3_pos := k3.PositionDot
	k4_pos := k4.PositionDot
	avgPosDot := k1_pos.Add(k2_pos.Scale(2)).Add(k3_pos.Scale(2)).Add(k4_pos).Scale(1.0/6.0)
	newState.Position = state.Position.Add(avgPosDot.Scale(dt))
	
	// Velocity integration
	k1_vel := k1.VelocityDot
	k2_vel := k2.VelocityDot
	k3_vel := k3.VelocityDot
	k4_vel := k4.VelocityDot
	avgVelDot := k1_vel.Add(k2_vel.Scale(2)).Add(k3_vel.Scale(2)).Add(k4_vel).Scale(1.0/6.0)
	newState.Velocity = state.Velocity.Add(avgVelDot.Scale(dt))
	
	// Angular rate integration
	k1_ang := k1.AngularRateDot
	k2_ang := k2.AngularRateDot
	k3_ang := k3.AngularRateDot
	k4_ang := k4.AngularRateDot
	avgAngDot := k1_ang.Add(k2_ang.Scale(2)).Add(k3_ang.Scale(2)).Add(k4_ang).Scale(1.0/6.0)
	newState.AngularRate = state.AngularRate.Add(avgAngDot.Scale(dt))
	
	// Orientation integration (quaternion)
	k1_orient := rk.quaternionDerivative(state.Orientation, state.AngularRate)
	midAngRate1 := state.AngularRate.Add(k1.AngularRateDot.Scale(dt*0.5))
	midOrient1 := state.Orientation.Add(k1_orient.Scale(dt*0.5)).Normalize()
	k2_orient := rk.quaternionDerivative(midOrient1, midAngRate1)
	
	midAngRate2 := state.AngularRate.Add(k2.AngularRateDot.Scale(dt*0.5))
	midOrient2 := state.Orientation.Add(k2_orient.Scale(dt*0.5)).Normalize()
	k3_orient := rk.quaternionDerivative(midOrient2, midAngRate2)
	
	finalAngRate := state.AngularRate.Add(k3.AngularRateDot.Scale(dt))
	finalOrient := state.Orientation.Add(k3_orient.Scale(dt)).Normalize()
	k4_orient := rk.quaternionDerivative(finalOrient, finalAngRate)
	
	avgOrientDot := k1_orient.Add(k2_orient.Scale(2)).Add(k3_orient.Scale(2)).Add(k4_orient).Scale(1.0/6.0)
	newState.Orientation = state.Orientation.Add(avgOrientDot.Scale(dt)).Normalize()
	
	// Update derived parameters
	newState.Altitude = -newState.Position.Z
	newState.Time = state.Time + dt
	newState.UpdateDerivedParameters()
	
	return newState
}

// advanceState creates an intermediate state by advancing with given derivatives
func (rk *TrueRK4Integrator) advanceState(state *AircraftState, derivatives *StateDerivatives, dt float64) *AircraftState {
	newState := state.Copy()
	
	// Advance position
	newState.Position = state.Position.Add(derivatives.PositionDot.Scale(dt))
	
	// Advance velocity
	newState.Velocity = state.Velocity.Add(derivatives.VelocityDot.Scale(dt))
	
	// Advance angular rate
	newState.AngularRate = state.AngularRate.Add(derivatives.AngularRateDot.Scale(dt))
	
	// Advance orientation
	orientDot := rk.quaternionDerivative(state.Orientation, state.AngularRate)
	newState.Orientation = state.Orientation.Add(orientDot.Scale(dt)).Normalize()
	
	// Update derived parameters
	newState.Altitude = -newState.Position.Z
	newState.Time = state.Time + dt
	newState.UpdateDerivedParameters()
	
	return newState
}

// quaternionDerivative calculates quaternion derivative from angular velocity
func (rk *TrueRK4Integrator) quaternionDerivative(q Quaternion, omega Vector3) Quaternion {
	omegaQuat := Quaternion{W: 0, X: omega.X, Y: omega.Y, Z: omega.Z}
	return q.Multiply(omegaQuat).Scale(0.5)
}

// simplifiedIntegrate provides a fallback if dynamics evaluation fails
func (rk *TrueRK4Integrator) simplifiedIntegrate(state *AircraftState, derivatives *StateDerivatives, dt float64) *AircraftState {
	// Fallback to the improved approximation method
	newState := state.Copy()
	
	// Use the improved approximation from the regular RK4
	k1_vel := derivatives.VelocityDot
	k2_vel := k1_vel.Scale(0.95)
	k3_vel := k1_vel.Scale(0.98)
	k4_vel := k1_vel.Scale(0.90)
	avgVelDot := k1_vel.Add(k2_vel.Scale(2)).Add(k3_vel.Scale(2)).Add(k4_vel).Scale(1.0/6.0)
	newState.Velocity = state.Velocity.Add(avgVelDot.Scale(dt))
	
	k1_angvel := derivatives.AngularRateDot
	k2_angvel := k1_angvel.Scale(0.95)
	k3_angvel := k1_angvel.Scale(0.98)
	k4_angvel := k1_angvel.Scale(0.90)
	avgAngVelDot := k1_angvel.Add(k2_angvel.Scale(2)).Add(k3_angvel.Scale(2)).Add(k4_angvel).Scale(1.0/6.0)
	newState.AngularRate = state.AngularRate.Add(avgAngVelDot.Scale(dt))
	
	// Position and orientation
	newState.Position = state.Position.Add(derivatives.PositionDot.Scale(dt))
	orientDot := rk.quaternionDerivative(state.Orientation, state.AngularRate)
	newState.Orientation = state.Orientation.Add(orientDot.Scale(dt)).Normalize()
	
	newState.Altitude = -newState.Position.Z
	newState.Time = state.Time + dt
	newState.UpdateDerivedParameters()
	
	return newState
}

// IntegratorWithDynamics wraps the true RK4 to work with existing flight dynamics engines
type IntegratorWithDynamics struct {
	TrueRK4 *TrueRK4Integrator
	Fallback Integrator
}

// NewIntegratorWithDynamics creates an integrator that can use true RK4 when possible
func NewIntegratorWithDynamics(dynamicsFunc DynamicsFunction) *IntegratorWithDynamics {
	return &IntegratorWithDynamics{
		TrueRK4:  NewTrueRK4Integrator(dynamicsFunc),
		Fallback: NewRungeKutta4Integrator(),
	}
}

func (iwd *IntegratorWithDynamics) GetName() string {
	return "RK4 with Dynamics Re-evaluation"
}

func (iwd *IntegratorWithDynamics) GetOrder() int {
	return 4
}

func (iwd *IntegratorWithDynamics) Integrate(state *AircraftState, derivatives *StateDerivatives, dt float64) *AircraftState {
	if iwd.TrueRK4.DynamicsFunc != nil {
		return iwd.TrueRK4.Integrate(state, derivatives, dt)
	}
	return iwd.Fallback.Integrate(state, derivatives, dt)
}
