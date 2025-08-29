// Flight Control System Property Manager
// Implements JSBSim-style property tree for component communication

package main

import (
	"fmt"
	"strings"
	"sync"
)

// PropertyManager manages the JSBSim-style property tree
// This is the central communication hub for all flight control components
type PropertyManager struct {
	properties map[string]float64
	aliases    map[string]string
	mutex      sync.RWMutex
	listeners  map[string][]PropertyListener
}

// PropertyListener interface for components that want to be notified of property changes
type PropertyListener interface {
	OnPropertyChanged(name string, value float64)
}

// NewPropertyManager creates a new property manager with common JSBSim properties
func NewPropertyManager() *PropertyManager {
	pm := &PropertyManager{
		properties: make(map[string]float64),
		aliases:    make(map[string]string),
		listeners:  make(map[string][]PropertyListener),
	}
	
	// Initialize common JSBSim properties with default values
	pm.initializeStandardProperties()
	
	return pm
}

// initializeStandardProperties sets up the standard JSBSim property tree
func (pm *PropertyManager) initializeStandardProperties() {
	// Flight Control System properties
	pm.properties["fcs/aileron-cmd-norm"] = 0.0
	pm.properties["fcs/elevator-cmd-norm"] = 0.0
	pm.properties["fcs/rudder-cmd-norm"] = 0.0
	pm.properties["fcs/throttle-cmd-norm"] = 0.0
	pm.properties["fcs/flap-cmd-norm"] = 0.0
	pm.properties["fcs/speedbrake-cmd-norm"] = 0.0
	pm.properties["fcs/gear-cmd-norm"] = 0.0
	
	// Control surface positions (actuator outputs)
	pm.properties["fcs/left-aileron-pos-rad"] = 0.0
	pm.properties["fcs/right-aileron-pos-rad"] = 0.0
	pm.properties["fcs/elevator-pos-rad"] = 0.0
	pm.properties["fcs/rudder-pos-rad"] = 0.0
	pm.properties["fcs/flap-pos-norm"] = 0.0
	pm.properties["fcs/speedbrake-pos-norm"] = 0.0
	
	// Pilot inputs (normalized -1 to +1)
	pm.properties["fcs/aileron-cmd-norm"] = 0.0
	pm.properties["fcs/elevator-cmd-norm"] = 0.0
	pm.properties["fcs/rudder-cmd-norm"] = 0.0
	pm.properties["fcs/throttle-cmd-norm"] = 0.0
	
	// Atmospheric properties
	pm.properties["atmosphere/density-slugft3"] = 0.002378
	pm.properties["atmosphere/pressure-psf"] = 2116.2
	pm.properties["atmosphere/temperature-R"] = 518.67
	pm.properties["atmosphere/rho"] = 1.225
	
	// Velocities
	pm.properties["velocities/vc-kts"] = 0.0
	pm.properties["velocities/ve-kts"] = 0.0
	pm.properties["velocities/vt-fps"] = 0.0
	pm.properties["velocities/mach"] = 0.0
	pm.properties["velocities/alpha-rad"] = 0.0
	pm.properties["velocities/beta-rad"] = 0.0
	
	// Angular rates
	pm.properties["velocities/p-rad_sec"] = 0.0
	pm.properties["velocities/q-rad_sec"] = 0.0
	pm.properties["velocities/r-rad_sec"] = 0.0
	
	// Position and attitude
	pm.properties["position/h-sl-ft"] = 0.0
	pm.properties["attitude/phi-rad"] = 0.0
	pm.properties["attitude/theta-rad"] = 0.0
	pm.properties["attitude/psi-rad"] = 0.0
	
	// Forces and moments
	pm.properties["forces/fbx-lbs"] = 0.0
	pm.properties["forces/fby-lbs"] = 0.0
	pm.properties["forces/fbz-lbs"] = 0.0
	pm.properties["moments/l-lbsft"] = 0.0
	pm.properties["moments/m-lbsft"] = 0.0
	pm.properties["moments/n-lbsft"] = 0.0
}

// Set updates a property value and notifies listeners
func (pm *PropertyManager) Set(name string, value float64) {
	pm.mutex.Lock()
	defer pm.mutex.Unlock()
	
	// Resolve aliases
	if alias, exists := pm.aliases[name]; exists {
		name = alias
	}
	
	oldValue, existed := pm.properties[name]
	pm.properties[name] = value
	
	// Notify listeners if value changed
	if !existed || oldValue != value {
		if listeners, exists := pm.listeners[name]; exists {
			for _, listener := range listeners {
				go listener.OnPropertyChanged(name, value)
			}
		}
	}
}

// Get retrieves a property value
func (pm *PropertyManager) Get(name string) float64 {
	pm.mutex.RLock()
	defer pm.mutex.RUnlock()
	
	// Resolve aliases
	if alias, exists := pm.aliases[name]; exists {
		name = alias
	}
	
	if value, exists := pm.properties[name]; exists {
		return value
	}
	
	// Return 0.0 for non-existent properties (JSBSim behavior)
	return 0.0
}

// GetSafe retrieves a property value with existence check
func (pm *PropertyManager) GetSafe(name string) (float64, bool) {
	pm.mutex.RLock()
	defer pm.mutex.RUnlock()
	
	// Resolve aliases
	if alias, exists := pm.aliases[name]; exists {
		name = alias
	}
	
	value, exists := pm.properties[name]
	return value, exists
}

// SetAlias creates an alias for a property
func (pm *PropertyManager) SetAlias(alias, target string) {
	pm.mutex.Lock()
	defer pm.mutex.Unlock()
	
	pm.aliases[alias] = target
}

// AddListener registers a listener for property changes
func (pm *PropertyManager) AddListener(propertyName string, listener PropertyListener) {
	pm.mutex.Lock()
	defer pm.mutex.Unlock()
	
	if pm.listeners[propertyName] == nil {
		pm.listeners[propertyName] = make([]PropertyListener, 0)
	}
	pm.listeners[propertyName] = append(pm.listeners[propertyName], listener)
}

// UpdateFromAircraftState synchronizes properties with aircraft state
func (pm *PropertyManager) UpdateFromAircraftState(state *AircraftState) {
	// Control inputs
	pm.Set("fcs/aileron-cmd-norm", state.Controls.Aileron)
	pm.Set("fcs/elevator-cmd-norm", state.Controls.Elevator)
	pm.Set("fcs/rudder-cmd-norm", state.Controls.Rudder)
	pm.Set("fcs/throttle-cmd-norm", state.Controls.Throttle)
	pm.Set("fcs/flap-cmd-norm", state.Controls.Flaps)
	
	// Control surface positions
	pm.Set("fcs/left-aileron-pos-rad", state.ControlSurfaces.AileronLeft)
	pm.Set("fcs/right-aileron-pos-rad", state.ControlSurfaces.AileronRight)
	pm.Set("fcs/elevator-pos-rad", state.ControlSurfaces.Elevator)
	pm.Set("fcs/rudder-pos-rad", state.ControlSurfaces.Rudder)
	
	// Atmospheric properties
	pm.Set("atmosphere/rho", state.Density)
	pm.Set("atmosphere/pressure-psf", state.Pressure*0.020885) // Pa to psf
	pm.Set("atmosphere/temperature-R", state.Temperature*1.8)  // K to R
	
	// Velocities and rates
	pm.Set("velocities/vt-fps", state.Velocity.Magnitude()*3.28084) // m/s to ft/s
	pm.Set("velocities/vc-kts", state.CalibratedAirspeed*1.94384)   // m/s to kts
	pm.Set("velocities/alpha-rad", 0.0) // TODO: Calculate angle of attack
	pm.Set("velocities/beta-rad", 0.0)  // TODO: Calculate sideslip angle
	
	// Angular rates
	pm.Set("velocities/p-rad_sec", state.AngularRate.X)
	pm.Set("velocities/q-rad_sec", state.AngularRate.Y)
	pm.Set("velocities/r-rad_sec", state.AngularRate.Z)
	
	// Position and attitude
	pm.Set("position/h-sl-ft", state.Altitude*3.28084) // m to ft
	pm.Set("attitude/phi-rad", 0.0)   // TODO: Calculate roll from quaternion
	pm.Set("attitude/theta-rad", 0.0) // TODO: Calculate pitch from quaternion
	pm.Set("attitude/psi-rad", 0.0)   // TODO: Calculate yaw from quaternion
}

// ApplyToAircraftState updates aircraft state from properties
func (pm *PropertyManager) ApplyToAircraftState(state *AircraftState) {
	// Update control surface positions from FCS outputs
	state.ControlSurfaces.AileronLeft = pm.Get("fcs/left-aileron-pos-rad")
	state.ControlSurfaces.AileronRight = pm.Get("fcs/right-aileron-pos-rad")
	state.ControlSurfaces.Elevator = pm.Get("fcs/elevator-pos-rad")
	state.ControlSurfaces.Rudder = pm.Get("fcs/rudder-pos-rad")
	
	// Note: Control inputs should flow: pilot → FCS → actuators → surfaces
	// So we don't update state.Controls from properties here
}

// ListProperties returns all property names (for debugging)
func (pm *PropertyManager) ListProperties() []string {
	pm.mutex.RLock()
	defer pm.mutex.RUnlock()
	
	var names []string
	for name := range pm.properties {
		names = append(names, name)
	}
	return names
}

// GetPropertiesWithPrefix returns all properties matching a prefix
func (pm *PropertyManager) GetPropertiesWithPrefix(prefix string) map[string]float64 {
	pm.mutex.RLock()
	defer pm.mutex.RUnlock()
	
	result := make(map[string]float64)
	for name, value := range pm.properties {
		if strings.HasPrefix(name, prefix) {
			result[name] = value
		}
	}
	return result
}

// String returns a formatted representation of the property manager
func (pm *PropertyManager) String() string {
	pm.mutex.RLock()
	defer pm.mutex.RUnlock()
	
	var sb strings.Builder
	sb.WriteString("PropertyManager {\n")
	
	// Group by category
	categories := map[string][]string{
		"fcs":         {},
		"velocities":  {},
		"attitude":    {},
		"position":    {},
		"atmosphere":  {},
		"forces":      {},
		"moments":     {},
	}
	
	for name := range pm.properties {
		for category := range categories {
			if strings.HasPrefix(name, category+"/") {
				categories[category] = append(categories[category], name)
				break
			}
		}
	}
	
	for category, names := range categories {
		if len(names) > 0 {
			sb.WriteString(fmt.Sprintf("  %s: {\n", category))
			for _, name := range names {
				sb.WriteString(fmt.Sprintf("    %s: %.6f\n", name, pm.properties[name]))
			}
			sb.WriteString("  }\n")
		}
	}
	
	sb.WriteString("}")
	return sb.String()
}
