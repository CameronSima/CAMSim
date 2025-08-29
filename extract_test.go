package main

import (
	"reflect"
	"strings"
	"testing"
)

// Test data representing a simplified P-51D configuration for testing
var testXMLData = `<?xml version="1.0"?>
<fdm_config name="P-51D Test" version="2.0" release="BETA">
	<fileheader>
		<author>Test Author</author>
		<filecreationdate>2024-01-01</filecreationdate>
		<version>1.0</version>
		<description>Test P-51D configuration</description>
		<reference refID="REF001" author="NACA" title="P-51 Report" date="1945"/>
		<reference refID="REF002" author="Boeing" title="Flight Test Data" date="1946"/>
	</fileheader>

	<metrics>
		<wingarea unit="FT2">235</wingarea>
		<wingspan unit="FT">37.1</wingspan>
		<chord unit="FT">6.6</chord>
		<htailarea unit="FT2">41</htailarea>
		<htailarm unit="FT">15</htailarm>
		<vtailarea unit="FT2">20</vtailarea>
		<vtailarm unit="FT">15</vtailarm>
		<location name="AERORP" unit="IN">
			<x>99</x>
			<y>0</y>
			<z>-26.5</z>
		</location>
		<location name="EYEPOINT" unit="IN">
			<x>95</x>
			<y>0</y>
			<z>30</z>
		</location>
	</metrics>

	<mass_balance>
		<ixx unit="SLUG*FT2">8031</ixx>
		<iyy unit="SLUG*FT2">9274</iyy>
		<izz unit="SLUG*FT2">14547</izz>
		<emptywt unit="LBS">7125</emptywt>
		<location name="CG" unit="IN">
			<x>98</x>
			<y>0</y>
			<z>-9</z>
		</location>
		<pointmass name="pilot">
			<weight unit="LBS">180</weight>
			<location name="POINTMASS" unit="IN">
				<x>98</x>
				<y>0</y>
				<z>15</z>
			</location>
		</pointmass>
		<pointmass name="fuel">
			<weight unit="LBS">500</weight>
			<location name="POINTMASS" unit="IN">
				<x>106</x>
				<y>0</y>
				<z>-10</z>
			</location>
		</pointmass>
	</mass_balance>

	<ground_reactions>
		<contact type="BOGEY" name="LEFT_MLG">
			<location unit="IN">
				<x>75.25</x>
				<y>-72</y>
				<z>-85</z>
			</location>
			<static_friction>0.8</static_friction>
			<dynamic_friction>0.5</dynamic_friction>
			<rolling_friction>0.02</rolling_friction>
			<spring_coeff unit="LBS/FT">9800</spring_coeff>
			<damping_coeff unit="LBS/FT/SEC">2500</damping_coeff>
			<max_steer unit="DEG">0.0</max_steer>
			<brake_group>LEFT</brake_group>
			<retractable>1</retractable>
		</contact>
		<contact type="BOGEY" name="RIGHT_MLG">
			<location unit="IN">
				<x>75.25</x>
				<y>72</y>
				<z>-85</z>
			</location>
			<static_friction>0.8</static_friction>
			<dynamic_friction>0.5</dynamic_friction>
			<rolling_friction>0.02</rolling_friction>
			<spring_coeff unit="LBS/FT">9800</spring_coeff>
			<damping_coeff unit="LBS/FT/SEC">2500</damping_coeff>
			<max_steer unit="DEG">0.0</max_steer>
			<brake_group>RIGHT</brake_group>
			<retractable>1</retractable>
		</contact>
	</ground_reactions>

	<propulsion>
		<engine file="Packard-V-1650-7" name="Main Engine">
			<location unit="IN">
				<x>36</x>
				<y>0</y>
				<z>0</z>
			</location>
			<orient unit="DEG">
				<roll>-4.0</roll>
				<pitch>2.5</pitch>
				<yaw>0</yaw>
			</orient>
			<feed>0</feed>
			<feed>1</feed>
			<thruster file="P51prop" name="Main Prop">
				<location unit="IN">
					<x>36</x>
					<y>0</y>
					<z>0</z>
				</location>
				<orient unit="DEG">
					<roll>-4.0</roll>
					<pitch>2.5</pitch>
					<yaw>0</yaw>
				</orient>
			</thruster>
		</engine>
		<tank type="FUEL" number="0">
			<location unit="IN">
				<x>106</x>
				<y>-80</y>
				<z>-9.675</z>
			</location>
			<capacity unit="LBS">553.84</capacity>
			<contents unit="LBS">396</contents>
			<temperature>59</temperature>
		</tank>
		<tank type="FUEL" number="1">
			<location unit="IN">
				<x>106</x>
				<y>80</y>
				<z>-9.675</z>
			</location>
			<capacity unit="LBS">553.84</capacity>
			<contents unit="LBS">396</contents>
			<temperature>59</temperature>
		</tank>
	</propulsion>

	<flight_control name="FCS: P51D">
		<channel name="Pitch">
			<component name="elevator-cmd-actuator" type="actuator">
				<input>fcs/elevator-cmd-norm</input>
				<rate_limit>5.0</rate_limit>
				<clipto>
					<min>-1.0</min>
					<max>0.6667</max>
				</clipto>
				<output>fcs/elevator-pos-norm</output>
			</component>
			<sensor name="pitch-sensor">
				<input>fcs/elevator-pos-norm</input>
				<lag>0.1</lag>
				<noise variation="uniform">0.01</noise>
				<quantization>
					<bits>12</bits>
					<min>-1.0</min>
					<max>1.0</max>
				</quantization>
				<drift_rate>0.001</drift_rate>
				<bias>0.0</bias>
			</sensor>
		</channel>
	</flight_control>

	<aerodynamics>
		<alphalimits unit="DEG">
			<min>-90</min>
			<max>90</max>
		</alphalimits>
		<axis name="DRAG">
			<function name="CDo">
				<description>Base drag coefficient</description>
				<product>
					<property>aero/qbar-psf</property>
					<property>metrics/Sw-sqft</property>
					<value>0.02</value>
				</product>
			</function>
		</axis>
		<function name="test-function">
			<description>Test function</description>
			<table name="test-table">
				<independentVar lookup="row">aero/alpha-deg</independentVar>
				<tableData>
					-10.0  0.1
					0.0    0.0
					10.0   0.1
				</tableData>
			</table>
		</function>
	</aerodynamics>

	<input port="1234" protocol="UDP"/>
	<output name="test-output" type="SOCKET" port="5678" protocol="TCP" rate="60"/>
</fdm_config>`

func TestExtractAllValues(t *testing.T) {
	// Parse the test XML
	config, err := ParseJSBSimConfig(strings.NewReader(testXMLData))
	if err != nil {
		t.Fatalf("Failed to parse test XML: %v", err)
	}

	// Extract all values
	values := ExtractAllValues(config)

	t.Run("Header Extraction", func(t *testing.T) {
		testHeaderExtraction(t, values)
	})

	t.Run("Metrics Extraction", func(t *testing.T) {
		testMetricsExtraction(t, values)
	})

	t.Run("Mass Balance Extraction", func(t *testing.T) {
		testMassBalanceExtraction(t, values)
	})

	t.Run("Ground Reactions Extraction", func(t *testing.T) {
		testGroundReactionsExtraction(t, values)
	})

	t.Run("Propulsion Extraction", func(t *testing.T) {
		testPropulsionExtraction(t, values)
	})

	t.Run("Flight Control Extraction", func(t *testing.T) {
		testFlightControlExtraction(t, values)
	})

	t.Run("Aerodynamics Extraction", func(t *testing.T) {
		testAerodynamicsExtraction(t, values)
	})

	t.Run("Input Output Extraction", func(t *testing.T) {
		testInputOutputExtraction(t, values)
	})
}

func testHeaderExtraction(t *testing.T, values map[string]interface{}) {
	// Test basic header fields
	assertEqual(t, values["header.author"], "Test Author")
	assertEqual(t, values["header.creation_date"], "2024-01-01")
	assertEqual(t, values["header.version"], "1.0")
	assertEqual(t, values["header.description"], "Test P-51D configuration")

	// Test references
	refs, ok := values["header.references"].([]map[string]string)
	if !ok {
		t.Errorf("Expected references to be []map[string]string, got %T", values["header.references"])
		return
	}

	if len(refs) != 2 {
		t.Errorf("Expected 2 references, got %d", len(refs))
		return
	}

	// Check first reference
	assertEqual(t, refs[0]["refID"], "REF001")
	assertEqual(t, refs[0]["author"], "NACA")
	assertEqual(t, refs[0]["title"], "P-51 Report")
	assertEqual(t, refs[0]["date"], "1945")

	// Check second reference
	assertEqual(t, refs[1]["refID"], "REF002")
	assertEqual(t, refs[1]["author"], "Boeing")
	assertEqual(t, refs[1]["title"], "Flight Test Data")
	assertEqual(t, refs[1]["date"], "1946")
}

func testMetricsExtraction(t *testing.T, values map[string]interface{}) {
	// Test wing metrics
	assertEqual(t, values["metrics.wing_area"], 235.0)
	assertEqual(t, values["metrics.wing_span"], 37.1)
	assertEqual(t, values["metrics.chord"], 6.6)
	assertEqual(t, values["metrics.htail_area"], 41.0)
	assertEqual(t, values["metrics.htail_arm"], 15.0)
	assertEqual(t, values["metrics.vtail_area"], 20.0)
	assertEqual(t, values["metrics.vtail_arm"], 15.0)

	// Test locations
	aerorp, ok := values["metrics.location.AERORP"].(map[string]float64)
	if !ok {
		t.Errorf("Expected AERORP location to be map[string]float64, got %T", values["metrics.location.AERORP"])
		return
	}
	assertEqual(t, aerorp["x"], 99.0)
	assertEqual(t, aerorp["y"], 0.0)
	assertEqual(t, aerorp["z"], -26.5)

	eyepoint, ok := values["metrics.location.EYEPOINT"].(map[string]float64)
	if !ok {
		t.Errorf("Expected EYEPOINT location to be map[string]float64, got %T", values["metrics.location.EYEPOINT"])
		return
	}
	assertEqual(t, eyepoint["x"], 95.0)
	assertEqual(t, eyepoint["y"], 0.0)
	assertEqual(t, eyepoint["z"], 30.0)
}

func testMassBalanceExtraction(t *testing.T, values map[string]interface{}) {
	// Test inertia values
	assertEqual(t, values["mass_balance.ixx"], 8031.0)
	assertEqual(t, values["mass_balance.iyy"], 9274.0)
	assertEqual(t, values["mass_balance.izz"], 14547.0)
	assertEqual(t, values["mass_balance.empty_mass"], 7125.0)

	// Test CG location
	cgLoc, ok := values["mass_balance.cg_location"].(map[string]float64)
	if !ok {
		t.Errorf("Expected CG location to be map[string]float64, got %T", values["mass_balance.cg_location"])
		return
	}
	assertEqual(t, cgLoc["x"], 98.0)
	assertEqual(t, cgLoc["y"], 0.0)
	assertEqual(t, cgLoc["z"], -9.0)

	// Test point masses
	pilot, ok := values["mass_balance.point_mass.0"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected pilot point mass to be map[string]interface{}, got %T", values["mass_balance.point_mass.0"])
		return
	}
	assertEqual(t, pilot["name"], "pilot")
	assertEqual(t, pilot["mass"], 180.0)

	pilotLoc, ok := pilot["location"].(map[string]float64)
	if !ok {
		t.Errorf("Expected pilot location to be map[string]float64, got %T", pilot["location"])
		return
	}
	assertEqual(t, pilotLoc["x"], 98.0)
	assertEqual(t, pilotLoc["y"], 0.0)
	assertEqual(t, pilotLoc["z"], 15.0)

	// Test second point mass
	fuel, ok := values["mass_balance.point_mass.1"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected fuel point mass to be map[string]interface{}, got %T", values["mass_balance.point_mass.1"])
		return
	}
	assertEqual(t, fuel["name"], "fuel")
	assertEqual(t, fuel["mass"], 500.0)
}

func testGroundReactionsExtraction(t *testing.T, values map[string]interface{}) {
	// Test left main landing gear
	leftMLG, ok := values["ground_reactions.contact.0"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected left MLG to be map[string]interface{}, got %T", values["ground_reactions.contact.0"])
		return
	}

	assertEqual(t, leftMLG["type"], "BOGEY")
	assertEqual(t, leftMLG["name"], "LEFT_MLG")
	assertEqual(t, leftMLG["static_friction"], 0.8)
	assertEqual(t, leftMLG["dynamic_friction"], 0.5)
	assertEqual(t, leftMLG["rolling_friction"], 0.02)
	assertEqual(t, leftMLG["spring_constant"], 9800.0)
	assertEqual(t, leftMLG["damper_constant"], 2500.0)
	assertEqual(t, leftMLG["max_steer"], 0.0)
	assertEqual(t, leftMLG["brake_group"], "LEFT")
	assertEqual(t, leftMLG["retractable"], 1)

	// Test location
	leftMLGLoc, ok := leftMLG["location"].(map[string]float64)
	if !ok {
		t.Errorf("Expected left MLG location to be map[string]float64, got %T", leftMLG["location"])
		return
	}
	assertEqual(t, leftMLGLoc["x"], 75.25)
	assertEqual(t, leftMLGLoc["y"], -72.0)
	assertEqual(t, leftMLGLoc["z"], -85.0)

	// Test right main landing gear exists
	rightMLG, ok := values["ground_reactions.contact.1"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected right MLG to be map[string]interface{}, got %T", values["ground_reactions.contact.1"])
		return
	}
	assertEqual(t, rightMLG["name"], "RIGHT_MLG")
	assertEqual(t, rightMLG["brake_group"], "RIGHT")
}

func testPropulsionExtraction(t *testing.T, values map[string]interface{}) {
	// Test engine
	engine, ok := values["propulsion.engine.0"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected engine to be map[string]interface{}, got %T", values["propulsion.engine.0"])
		return
	}

	assertEqual(t, engine["file"], "Packard-V-1650-7")
	assertEqual(t, engine["name"], "Main Engine")

	// Test engine location
	engineLoc, ok := engine["location"].(map[string]float64)
	if !ok {
		t.Errorf("Expected engine location to be map[string]float64, got %T", engine["location"])
		return
	}
	assertEqual(t, engineLoc["x"], 36.0)
	assertEqual(t, engineLoc["y"], 0.0)
	assertEqual(t, engineLoc["z"], 0.0)

	// Test engine orientation
	engineOrient, ok := engine["orientation"].(map[string]float64)
	if !ok {
		t.Errorf("Expected engine orientation to be map[string]float64, got %T", engine["orientation"])
		return
	}
	assertEqual(t, engineOrient["roll"], -4.0)
	assertEqual(t, engineOrient["pitch"], 2.5)
	assertEqual(t, engineOrient["yaw"], 0.0)

	// Test feed
	feed, ok := engine["feed"].([]int)
	if !ok {
		t.Errorf("Expected engine feed to be []int, got %T", engine["feed"])
		return
	}
	if len(feed) != 2 {
		t.Errorf("Expected 2 feed tanks, got %d", len(feed))
		return
	}
	assertEqual(t, feed[0], 0)
	assertEqual(t, feed[1], 1)

	// Test thruster
	thruster, ok := engine["thruster"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected thruster to be map[string]interface{}, got %T", engine["thruster"])
		return
	}
	assertEqual(t, thruster["file"], "P51prop")
	assertEqual(t, thruster["name"], "Main Prop")

	// Test tanks
	tank0, ok := values["propulsion.tank.0"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected tank 0 to be map[string]interface{}, got %T", values["propulsion.tank.0"])
		return
	}
	assertEqual(t, tank0["type"], "FUEL")
	assertEqual(t, tank0["number"], 0)
	assertEqual(t, tank0["capacity"], 553.84)
	assertEqual(t, tank0["contents"], 396.0)
	assertEqual(t, tank0["temperature"], 59.0)

	tank1, ok := values["propulsion.tank.1"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected tank 1 to be map[string]interface{}, got %T", values["propulsion.tank.1"])
		return
	}
	assertEqual(t, tank1["number"], 1)
}

func testFlightControlExtraction(t *testing.T, values map[string]interface{}) {
	// Test flight control name
	assertEqual(t, values["flight_control.name"], "FCS: P51D")

	// Test component
	comp, ok := values["flight_control.channel.Pitch.component.0"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected component to be map[string]interface{}, got %T", values["flight_control.channel.Pitch.component.0"])
		return
	}

	assertEqual(t, comp["name"], "elevator-cmd-actuator")
	assertEqual(t, comp["type"], "actuator")
	assertEqual(t, comp["output"], "fcs/elevator-pos-norm")
	assertEqual(t, comp["clip_min"], -1.0)
	assertEqual(t, comp["clip_max"], 0.6667)

	// Test sensor
	sensor, ok := values["flight_control.channel.Pitch.sensor.0"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected sensor to be map[string]interface{}, got %T", values["flight_control.channel.Pitch.sensor.0"])
		return
	}

	assertEqual(t, sensor["name"], "pitch-sensor")
	assertEqual(t, sensor["input"], "fcs/elevator-pos-norm")
	assertEqual(t, sensor["lag"], 0.1)
	assertEqual(t, sensor["drift_rate"], 0.001)
	assertEqual(t, sensor["bias"], 0.0)

	// Test noise
	noise, ok := sensor["noise"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected noise to be map[string]interface{}, got %T", sensor["noise"])
		return
	}
	assertEqual(t, noise["variation"], "uniform")
	assertEqual(t, noise["value"], 0.01)

	// Test quantization
	quant, ok := sensor["quantization"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected quantization to be map[string]interface{}, got %T", sensor["quantization"])
		return
	}
	assertEqual(t, quant["bits"], 12)
	assertEqual(t, quant["min"], -1.0)
	assertEqual(t, quant["max"], 1.0)
}

func testAerodynamicsExtraction(t *testing.T, values map[string]interface{}) {
	// Test alpha limits
	alphaLimits, ok := values["aerodynamics.alpha_limits"].(map[string]float64)
	if !ok {
		t.Errorf("Expected alpha limits to be map[string]float64, got %T", values["aerodynamics.alpha_limits"])
		return
	}
	assertEqual(t, alphaLimits["min"], -90.0)
	assertEqual(t, alphaLimits["max"], 90.0)

	// Test axis function
	axisFn, ok := values["aerodynamics.axis.DRAG.function.0"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected axis function to be map[string]interface{}, got %T", values["aerodynamics.axis.DRAG.function.0"])
		return
	}
	assertEqual(t, axisFn["name"], "CDo")
	assertEqual(t, axisFn["description"], "Base drag coefficient")
	assertEqual(t, axisFn["operation"], "product")

	// Test standalone function with table
	standaloneFn, ok := values["aerodynamics.function.0"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected standalone function to be map[string]interface{}, got %T", values["aerodynamics.function.0"])
		return
	}
	assertEqual(t, standaloneFn["name"], "test-function")
	assertEqual(t, standaloneFn["description"], "Test function")

	// Test table data
	table, ok := standaloneFn["table"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected table to be map[string]interface{}, got %T", standaloneFn["table"])
		return
	}
	assertEqual(t, table["name"], "test-table")
	assertEqual(t, table["dimension"], 1)

	// Test independent variables
	indVars, ok := table["independent_vars"].([]map[string]string)
	if !ok {
		t.Errorf("Expected independent vars to be []map[string]string, got %T", table["independent_vars"])
		return
	}
	if len(indVars) != 1 {
		t.Errorf("Expected 1 independent variable, got %d", len(indVars))
		return
	}
	assertEqual(t, indVars[0]["lookup"], "row")
	assertEqual(t, indVars[0]["value"], "aero/alpha-deg")

	// Test 1D table data
	data1D, ok := table["data_1d"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected 1D data to be map[string]interface{}, got %T", table["data_1d"])
		return
	}

	indices, ok := data1D["indices"].([]float64)
	if !ok {
		t.Errorf("Expected indices to be []float64, got %T", data1D["indices"])
		return
	}
	if len(indices) != 3 {
		t.Errorf("Expected 3 indices, got %d", len(indices))
		return
	}
	assertEqual(t, indices[0], -10.0)
	assertEqual(t, indices[1], 0.0)
	assertEqual(t, indices[2], 10.0)

	values1D, ok := data1D["values"].([]float64)
	if !ok {
		t.Errorf("Expected values to be []float64, got %T", data1D["values"])
		return
	}
	if len(values1D) != 3 {
		t.Errorf("Expected 3 values, got %d", len(values1D))
		return
	}
	assertEqual(t, values1D[0], 0.1)
	assertEqual(t, values1D[1], 0.0)
	assertEqual(t, values1D[2], 0.1)
}

func testInputOutputExtraction(t *testing.T, values map[string]interface{}) {
	// Test input
	input, ok := values["input"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected input to be map[string]interface{}, got %T", values["input"])
		return
	}
	assertEqual(t, input["port"], 1234)
	assertEqual(t, input["protocol"], "UDP")

	// Test output
	output, ok := values["output"].(map[string]interface{})
	if !ok {
		t.Errorf("Expected output to be map[string]interface{}, got %T", values["output"])
		return
	}
	assertEqual(t, output["name"], "test-output")
	assertEqual(t, output["type"], "SOCKET")
	assertEqual(t, output["port"], 5678)
	assertEqual(t, output["protocol"], "TCP")
	assertEqual(t, output["rate"], 60)
}

// Helper function for assertions
func assertEqual(t *testing.T, actual, expected interface{}) {
	t.Helper()
	if !reflect.DeepEqual(actual, expected) {
		t.Errorf("Expected %v (type %T), got %v (type %T)", expected, expected, actual, actual)
	}
}

// Test with the actual P-51D XML file
func TestExtractAllValuesRealFile(t *testing.T) {
	// Read the actual P-51D file
	data := `<?xml version="1.0"?>
<fdm_config name="P-51D (JSBSim)" version="2.0" release="BETA">
    <fileheader>
        <author> Aeromatic / Jon Berndt / DATCOM / Hal V. Engel</author>
        <filecreationdate> 2001-01-01 </filecreationdate>
        <version> $Revision: 1.16 $ </version>
        <description> Advanced model of P-51D fighter </description>
    </fileheader>
    <metrics>
        <wingarea unit="FT2"> 235 </wingarea>
        <wingspan unit="FT"> 37.1 </wingspan>
    </metrics>
</fdm_config>`

	config, err := ParseJSBSimConfig(strings.NewReader(data))
	if err != nil {
		t.Fatalf("Failed to parse real XML: %v", err)
	}

	values := ExtractAllValues(config)

	// Test that header values are extracted correctly
	assertEqual(t, values["header.author"], " Aeromatic / Jon Berndt / DATCOM / Hal V. Engel")
	assertEqual(t, values["header.creation_date"], " 2001-01-01 ")
	assertEqual(t, values["header.version"], " $Revision: 1.16 $ ")
	assertEqual(t, values["header.description"], " Advanced model of P-51D fighter ")

	// Test metrics
	assertEqual(t, values["metrics.wing_area"], 235.0)
	assertEqual(t, values["metrics.wing_span"], 37.1)
}

// Benchmark the extraction process
func BenchmarkExtractAllValues(b *testing.B) {
	config, err := ParseJSBSimConfig(strings.NewReader(testXMLData))
	if err != nil {
		b.Fatalf("Failed to parse test XML: %v", err)
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = ExtractAllValues(config)
	}
}

// Test error handling
func TestExtractAllValuesWithNilConfig(t *testing.T) {
	defer func() {
		if r := recover(); r == nil {
			t.Errorf("Expected panic with nil config, but didn't panic")
		}
	}()
	ExtractAllValues(nil)
}

// Test that all expected sections are present in extraction
func TestExtractAllValuesCompleteness(t *testing.T) {
	config, err := ParseJSBSimConfig(strings.NewReader(testXMLData))
	if err != nil {
		t.Fatalf("Failed to parse test XML: %v", err)
	}

	values := ExtractAllValues(config)

	// Expected top-level sections that should be present
	expectedSections := []string{
		"header.author",
		"metrics.wing_area", 
		"mass_balance.ixx",
		"ground_reactions.contact.0",
		"propulsion.engine.0",
		"flight_control.name",
		"aerodynamics.alpha_limits",
		"input",
		"output",
	}

	for _, section := range expectedSections {
		if _, exists := values[section]; !exists {
			t.Errorf("Expected section '%s' to be present in extracted values", section)
		}
	}

	// Log total number of extracted values for verification
	t.Logf("Total extracted values: %d", len(values))
	
	// Ensure we're extracting a reasonable number of values
	if len(values) < 20 {
		t.Errorf("Expected at least 20 extracted values, got %d", len(values))
	}
}
