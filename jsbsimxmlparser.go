package main

import (
	"encoding/xml"
	"fmt"
	"io"
	"strconv"
	"strings"
)

// Unit conversion constants
const (
	FT_TO_M     = 0.3048
	M_TO_FT     = 3.28084
	FT2_TO_M2   = 0.092903
	M2_TO_FT2   = 10.7639
	IN_TO_FT    = 0.083333
	FT_TO_IN    = 12.0
	LB_TO_KG    = 0.453592
	KG_TO_LB    = 2.20462
	RAD_TO_DEG  = 57.2958
	DEG_TO_RAD  = 0.0174533
	KTS_TO_FPS  = 1.68781
	FPS_TO_KTS  = 0.592484
	HP_TO_W     = 745.7
	W_TO_HP     = 0.00134102
)

// JSBSimConfig represents the root configuration
type JSBSimConfig struct {
	XMLName         xml.Name         `xml:"fdm_config"`
	Name            string           `xml:"name,attr"`
	Version         string           `xml:"version,attr"`
	ReleaseLevel    string           `xml:"release,attr"`
	Header          *Header          `xml:"fileheader"`
	Metrics         *Metrics         `xml:"metrics"`
	MassBalance     *MassBalance     `xml:"mass_balance"`
	GroundReactions *GroundReactions `xml:"ground_reactions"`
	Propulsion      *Propulsion      `xml:"propulsion"`
	FlightControl   *FlightControl   `xml:"flight_control"`
	Autopilot       *FlightControl   `xml:"autopilot"`
	Aerodynamics    *Aerodynamics    `xml:"aerodynamics"`
	Input           *Input           `xml:"input"`
	Output          *Output          `xml:"output"`
	SystemControl   *SystemControl   `xml:"system"`
}

// Header contains administrative and source information
type Header struct {
	Author           string       `xml:"author"`
	FileCreationDate string       `xml:"filecreationdate"`
	Description      string       `xml:"description"`
	Version          string       `xml:"version"`
	References       []*Reference `xml:"reference"`
}

// Reference contains reference information
type Reference struct {
	RefID  string `xml:"refID,attr"`
	Author string `xml:"author,attr"`
	Title  string `xml:"title,attr"`
	Date   string `xml:"date,attr"`
}

// Metrics contains geometric parameters
type Metrics struct {
	WingArea    *Measurement `xml:"wingarea"`
	WingSpan    *Measurement `xml:"wingspan"`
	Chord       *Measurement `xml:"chord"`
	HTailArea   *Measurement `xml:"htailarea"`
	HTailArm    *Measurement `xml:"htailarm"`
	VTailArea   *Measurement `xml:"vtailarea"`
	VTailArm    *Measurement `xml:"vtailarm"`
	Location    []*Location  `xml:"location"`
}

// Measurement represents a value with optional unit
type Measurement struct {
	Unit  string  `xml:"unit,attr"`
	Value float64 `xml:",chardata"`
}

// Location represents a 3D position
type Location struct {
	Name string  `xml:"name,attr"`
	Unit string  `xml:"unit,attr"`
	X    float64 `xml:"x"`
	Y    float64 `xml:"y"`
	Z    float64 `xml:"z"`
}

// MassBalance contains mass and inertia information
type MassBalance struct {
	IXX        *Measurement  `xml:"ixx"`
	IYY        *Measurement  `xml:"iyy"`
	IZZ        *Measurement  `xml:"izz"`
	IXY        *Measurement  `xml:"ixy"`
	IXZ        *Measurement  `xml:"ixz"`
	IYZ        *Measurement  `xml:"iyz"`
	EmptyMass  *Measurement  `xml:"emptywt"`
	Location   *Location     `xml:"location"`
	PointMass  []*PointMass  `xml:"pointmass"`
}

// PointMass represents a concentrated mass
type PointMass struct {
	Name     string       `xml:"name,attr"`
	Mass     *Measurement `xml:"weight"`
	Location *Location    `xml:"location"`
}

// GroundReactions contains landing gear and contact points
type GroundReactions struct {
	Contact []*Contact `xml:"contact"`
}

// Contact represents a ground contact point (landing gear, etc.)
type Contact struct {
	Type             string       `xml:"type,attr"`
	Name             string       `xml:"name,attr"`
	Location         *Location    `xml:"location"`
	StaticFriction   float64      `xml:"static_friction"`
	DynamicFriction  float64      `xml:"dynamic_friction"`
	RollingFriction  float64      `xml:"rolling_friction"`
	Spring           *Spring      `xml:"spring"`
	Damper           *Damper      `xml:"damper"`
	SpringCoeff      *Measurement `xml:"spring_coeff"`
	DampingCoeff     *Measurement `xml:"damping_coeff"`
	MaxSteer         *Measurement `xml:"max_steer"`
	BrakeGroup       string       `xml:"brake_group"`
	Retractable      int          `xml:"retractable"`
}

// Spring represents spring characteristics
type Spring struct {
	Type     string  `xml:"type,attr"`
	Constant float64 `xml:",chardata"`
}

// Damper represents damping characteristics
type Damper struct {
	Type     string  `xml:"type,attr"`
	Constant float64 `xml:",chardata"`
}

// Propulsion contains engine and tank definitions
type Propulsion struct {
	Engine []*Engine `xml:"engine"`
	Tank   []*Tank   `xml:"tank"`
}

// Engine represents an engine
type Engine struct {
	File       string     `xml:"file,attr"`
	Name       string     `xml:"name,attr"`
	Location   *Location  `xml:"location"`
	Orient     *Orient    `xml:"orient"`
	Feed       []int      `xml:"feed"`
	Thruster   *Thruster  `xml:"thruster"`
}

// Orient represents orientation angles
type Orient struct {
	Unit  string  `xml:"unit,attr"`
	Pitch float64 `xml:"pitch"`
	Roll  float64 `xml:"roll"`
	Yaw   float64 `xml:"yaw"`
}

// Thruster represents thrust generation
type Thruster struct {
	File     string    `xml:"file,attr"`
	Name     string    `xml:"name,attr"`
	Location *Location `xml:"location"`
	Orient   *Orient   `xml:"orient"`
}

// Tank represents a fuel/oxidizer tank
type Tank struct {
	Type        string       `xml:"type,attr"`
	Number      int          `xml:"number,attr"`
	Location    *Location    `xml:"location"`
	Capacity    *Measurement `xml:"capacity"`
	Contents    *Measurement `xml:"contents"`
	Temperature float64      `xml:"temperature"`
}

// FlightControl contains flight control system definition
type FlightControl struct {
	Name      string      `xml:"name,attr"`
	Property  []string    `xml:"property"`
	RateGroup []*RateGroup `xml:"rate_group"`
	Channel   []*Channel   `xml:"channel"`
}

// RateGroup defines execution rate for components
type RateGroup struct {
	Name   string  `xml:"name,attr"`
	RateHz float64 `xml:"rate_Hz,attr"`
}

// Channel groups related components
type Channel struct {
	Name      string       `xml:"name,attr"`
	Component []*Component `xml:"component"`
	Sensor    []*Sensor    `xml:"sensor"`
}

// Component represents a flight control component
type Component struct {
	Name      string     `xml:"name,attr"`
	Type      string     `xml:"type,attr"`
	RateGroup string     `xml:"rate_group,attr"`
	Input     []string   `xml:"input"`
	Output    string     `xml:"output"`
	Gain      float64    `xml:"gain"`
	Function  *Function  `xml:"function"`
	Clipto    *Clipto    `xml:"clipto"`
	Test      []*Test    `xml:"test"`
	Default   *Default   `xml:"default"`
	C1        float64    `xml:"c1"`
	C2        float64    `xml:"c2"`
	C3        float64    `xml:"c3"`
	C4        float64    `xml:"c4"`
	C5        float64    `xml:"c5"`
	C6        float64    `xml:"c6"`
	Traverse  *Traverse  `xml:"traverse"`
	Width     float64    `xml:"width"`
}

// Sensor represents a sensor with noise and lag
type Sensor struct {
	Name         string        `xml:"name,attr"`
	RateGroup    string        `xml:"rate_group,attr"`
	Input        string        `xml:"input"`
	Lag          float64       `xml:"lag"`
	Noise        *Noise        `xml:"noise"`
	Quantization *Quantization `xml:"quantization"`
	DriftRate    float64       `xml:"drift_rate"`
	Bias         float64       `xml:"bias"`
}

// Noise represents sensor noise characteristics
type Noise struct {
	Variation string  `xml:"variation,attr"`
	Value     float64 `xml:",chardata"`
}

// Quantization represents digital quantization
type Quantization struct {
	Bits int     `xml:"bits"`
	Min  float64 `xml:"min"`
	Max  float64 `xml:"max"`
}

// Test represents a conditional test
type Test struct {
	Logic string  `xml:"logic,attr"`
	Value string  `xml:"value,attr"`
	Test  string  `xml:",chardata"`
}

// Default represents a default value
type Default struct {
	Value string `xml:"value,attr"`
}

// Clipto represents output limiting
type Clipto struct {
	Min float64 `xml:"min"`
	Max float64 `xml:"max"`
}

// Traverse represents kinematic traversal
type Traverse struct {
	Setting []*Setting `xml:"setting"`
}

// Setting represents a kinematic setting
type Setting struct {
	Position float64 `xml:"position"`
	Time     float64 `xml:"time"`
}

// Aerodynamics contains aerodynamic coefficients
type Aerodynamics struct {
	AlphaLimits *AlphaLimits `xml:"alphalimits"`
	Axis        []*Axis      `xml:"axis"`
	Function    []*Function  `xml:"function"`
}

// AlphaLimits defines angle of attack limits
type AlphaLimits struct {
	Unit string  `xml:"unit,attr"`
	Min  float64 `xml:"min"`
	Max  float64 `xml:"max"`
}

// Axis represents an aerodynamic axis
type Axis struct {
	Name     string      `xml:"name,attr"`
	Function []*Function `xml:"function"`
}

// Function represents a mathematical function
type Function struct {
	Name        string      `xml:"name,attr"`
	Description string      `xml:"description"`
	Product     *Operation  `xml:"product"`
	Difference  *Operation  `xml:"difference"`
	Sum         *Operation  `xml:"sum"`
	Quotient    *Operation  `xml:"quotient"`
	Pow         *Operation  `xml:"pow"`
	Abs         *Operation  `xml:"abs"`
	Sin         *Operation  `xml:"sin"`
	Cos         *Operation  `xml:"cos"`
	Tan         *Operation  `xml:"tan"`
	Asin        *Operation  `xml:"asin"`
	Acos        *Operation  `xml:"acos"`
	Atan        *Operation  `xml:"atan"`
	Table       *Table      `xml:"table"`
}

// Operation represents a mathematical operation
type Operation struct {
	Property   []string    `xml:"property"`
	Value      []float64   `xml:"value"`
	Table      *Table      `xml:"table"`
	Product    *Operation  `xml:"product"`
	Difference *Operation  `xml:"difference"`
	Sum        *Operation  `xml:"sum"`
	Quotient   *Operation  `xml:"quotient"`
	Pow        *Operation  `xml:"pow"`
	Abs        *Operation  `xml:"abs"`
	Sin        *Operation  `xml:"sin"`
	Cos        *Operation  `xml:"cos"`
	Tan        *Operation  `xml:"tan"`
	Asin       *Operation  `xml:"asin"`
	Acos       *Operation  `xml:"acos"`
	Atan       *Operation  `xml:"atan"`
}

// Table represents a lookup table
type Table struct {
	Name           string           `xml:"name,attr"`
	IndependentVar []*IndependentVar `xml:"independentVar"`
	TableData      []*TableData      `xml:"tableData"`
}

// IndependentVar represents an independent variable for table lookup
type IndependentVar struct {
	Lookup string `xml:"lookup,attr"`
	Value  string `xml:",chardata"`
}

// TableData represents table data
type TableData struct {
	Breakpoint  string `xml:"breakpoint,attr"`
	BreakPoint  string `xml:"breakPoint,attr"`  // Support both variants
	Data        string `xml:",innerxml"`
}

// GetBreakpoint returns the breakpoint value, handling both naming conventions
func (td *TableData) GetBreakpoint() string {
	if td.Breakpoint != "" {
		return td.Breakpoint
	}
	return td.BreakPoint
}

// Input defines input interfaces
type Input struct {
	Port     int    `xml:"port,attr"`
	Protocol string `xml:"protocol,attr"`
}

// Output defines output interfaces
type Output struct {
	Name     string `xml:"name,attr"`
	Type     string `xml:"type,attr"`
	Port     int    `xml:"port,attr"`
	Protocol string `xml:"protocol,attr"`
	Rate     int    `xml:"rate,attr"`
}

// SystemControl represents system control definitions
type SystemControl struct {
	Name     string      `xml:"name,attr"`
	File     string      `xml:"file,attr"`
	Property []string    `xml:"property"`
	Channel  []*Channel  `xml:"channel"`
}

// ParseJSBSimConfig parses a JSBSim XML configuration
func ParseJSBSimConfig(r io.Reader) (*JSBSimConfig, error) {
	decoder := xml.NewDecoder(r)
	config := &JSBSimConfig{}
	
	if err := decoder.Decode(config); err != nil {
		return nil, fmt.Errorf("failed to parse JSBSim config: %w", err)
	}
	
	// Post-process to handle unit conversions
	if config.Metrics != nil {
		convertMetrics(config.Metrics)
	}
	if config.MassBalance != nil {
		convertMassBalance(config.MassBalance)
	}
	
	return config, nil
}

// convertMetrics converts metric units to standard units
func convertMetrics(m *Metrics) {
	if m.WingArea != nil {
		m.WingArea.Value = convertToStandardUnit(m.WingArea.Value, m.WingArea.Unit, "area")
	}
	if m.WingSpan != nil {
		m.WingSpan.Value = convertToStandardUnit(m.WingSpan.Value, m.WingSpan.Unit, "length")
	}
	if m.Chord != nil {
		m.Chord.Value = convertToStandardUnit(m.Chord.Value, m.Chord.Unit, "length")
	}
	if m.HTailArea != nil {
		m.HTailArea.Value = convertToStandardUnit(m.HTailArea.Value, m.HTailArea.Unit, "area")
	}
	if m.HTailArm != nil {
		m.HTailArm.Value = convertToStandardUnit(m.HTailArm.Value, m.HTailArm.Unit, "length")
	}
	if m.VTailArea != nil {
		m.VTailArea.Value = convertToStandardUnit(m.VTailArea.Value, m.VTailArea.Unit, "area")
	}
	if m.VTailArm != nil {
		m.VTailArm.Value = convertToStandardUnit(m.VTailArm.Value, m.VTailArm.Unit, "length")
	}
}

// convertMassBalance converts mass/inertia units
func convertMassBalance(mb *MassBalance) {
	if mb.IXX != nil {
		mb.IXX.Value = convertToStandardUnit(mb.IXX.Value, mb.IXX.Unit, "inertia")
	}
	if mb.IYY != nil {
		mb.IYY.Value = convertToStandardUnit(mb.IYY.Value, mb.IYY.Unit, "inertia")
	}
	if mb.IZZ != nil {
		mb.IZZ.Value = convertToStandardUnit(mb.IZZ.Value, mb.IZZ.Unit, "inertia")
	}
	if mb.IXY != nil {
		mb.IXY.Value = convertToStandardUnit(mb.IXY.Value, mb.IXY.Unit, "inertia")
	}
	if mb.IXZ != nil {
		mb.IXZ.Value = convertToStandardUnit(mb.IXZ.Value, mb.IXZ.Unit, "inertia")
	}
	if mb.IYZ != nil {
		mb.IYZ.Value = convertToStandardUnit(mb.IYZ.Value, mb.IYZ.Unit, "inertia")
	}
	if mb.EmptyMass != nil {
		mb.EmptyMass.Value = convertToStandardUnit(mb.EmptyMass.Value, mb.EmptyMass.Unit, "mass")
	}
}

// convertToStandardUnit converts a value to standard units
func convertToStandardUnit(value float64, unit string, unitType string) float64 {
	if unit == "" {
		return value // Already in standard units
	}
	
	switch unitType {
	case "length":
		switch strings.ToUpper(unit) {
		case "M":
			return value * M_TO_FT
		case "IN":
			return value * IN_TO_FT
		}
	case "area":
		switch strings.ToUpper(unit) {
		case "M2":
			return value * M2_TO_FT2
		}
	case "mass":
		switch strings.ToUpper(unit) {
		case "KG":
			return value * KG_TO_LB
		}
	case "inertia":
		switch strings.ToUpper(unit) {
		case "KG-M2", "KG*M2":
			return value * KG_TO_LB * M_TO_FT * M_TO_FT
		}
	case "angle":
		switch strings.ToUpper(unit) {
		case "DEG":
			return value * DEG_TO_RAD
		}
	case "velocity":
		switch strings.ToUpper(unit) {
		case "KTS":
			return value * KTS_TO_FPS
		}
	case "power":
		switch strings.ToUpper(unit) {
		case "HP":
			return value * HP_TO_W
		}
	}
	
	return value
}

// ParseTable parses table data into a usable format
func ParseTable(t *Table) (*ParsedTable, error) {
	pt := &ParsedTable{
		Name:           t.Name,
		IndependentVars: make([]string, len(t.IndependentVar)),
		LookupTypes:    make([]string, len(t.IndependentVar)),
	}
	
	for i, iv := range t.IndependentVar {
		pt.IndependentVars[i] = strings.TrimSpace(iv.Value)
		pt.LookupTypes[i] = iv.Lookup
	}
	
	// Parse table data based on dimensions
	if len(t.IndependentVar) == 1 {
		// 1D table
		pt.Dimension = 1
		pt.Data1D = parse1DTableData(t.TableData[0].Data)
	} else if len(t.IndependentVar) == 2 {
		// 2D table
		pt.Dimension = 2
		pt.Data2D = parse2DTableData(t.TableData[0].Data)
	} else if len(t.IndependentVar) == 3 {
		// 3D table
		pt.Dimension = 3
		pt.Data3D = make([]*Table2D, len(t.TableData))
		for i, td := range t.TableData {
			bp, _ := strconv.ParseFloat(td.GetBreakpoint(), 64)
			parsed2D := parse2DTableData(td.Data)
			pt.Data3D[i] = &Table2D{
				Breakpoint: bp,
				RowIndices: parsed2D.RowIndices,
				ColIndices: parsed2D.ColIndices,
				Data:       parsed2D.Data,
			}
		}
	}
	
	return pt, nil
}

// ParsedTable represents a parsed table
type ParsedTable struct {
	Name            string
	Dimension       int
	IndependentVars []string
	LookupTypes     []string
	Data1D          *Table1D
	Data2D          *Table2D
	Data3D          []*Table2D
}

// Table1D represents a 1D table
type Table1D struct {
	Indices []float64
	Values  []float64
}

// Table2D represents a 2D table
type Table2D struct {
	Breakpoint float64
	RowIndices []float64
	ColIndices []float64
	Data       [][]float64
}

// parse1DTableData parses 1D table data
func parse1DTableData(data string) *Table1D {
	lines := strings.Split(strings.TrimSpace(data), "\n")
	t := &Table1D{
		Indices: make([]float64, 0, len(lines)),
		Values:  make([]float64, 0, len(lines)),
	}
	
	for _, line := range lines {
		fields := strings.Fields(line)
		if len(fields) >= 2 {
			idx, _ := strconv.ParseFloat(fields[0], 64)
			val, _ := strconv.ParseFloat(fields[1], 64)
			t.Indices = append(t.Indices, idx)
			t.Values = append(t.Values, val)
		}
	}
	
	return t
}

// parse2DTableData parses 2D table data
func parse2DTableData(data string) *Table2D {
	lines := strings.Split(strings.TrimSpace(data), "\n")
	if len(lines) < 2 {
		return nil
	}
	
	// Parse column indices from first line
	colFields := strings.Fields(lines[0])
	t := &Table2D{
		ColIndices: make([]float64, 0, len(colFields)),
		RowIndices: make([]float64, 0, len(lines)-1),
		Data:       make([][]float64, 0, len(lines)-1),
	}
	
	for _, field := range colFields {
		val, _ := strconv.ParseFloat(field, 64)
		t.ColIndices = append(t.ColIndices, val)
	}
	
	// Parse data rows
	for i := 1; i < len(lines); i++ {
		fields := strings.Fields(lines[i])
		if len(fields) > 0 {
			rowIdx, _ := strconv.ParseFloat(fields[0], 64)
			t.RowIndices = append(t.RowIndices, rowIdx)
			
			row := make([]float64, 0, len(fields)-1)
			for j := 1; j < len(fields); j++ {
				val, _ := strconv.ParseFloat(fields[j], 64)
				row = append(row, val)
			}
			t.Data = append(t.Data, row)
		}
	}
	
	return t
}

// InterpolateTable performs table interpolation
func InterpolateTable(pt *ParsedTable, inputs ...float64) (float64, error) {
	switch pt.Dimension {
	case 1:
		if len(inputs) != 1 {
			return 0, fmt.Errorf("1D table requires 1 input, got %d", len(inputs))
		}
		return interpolate1D(pt.Data1D, inputs[0]), nil
	case 2:
		if len(inputs) != 2 {
			return 0, fmt.Errorf("2D table requires 2 inputs, got %d", len(inputs))
		}
		return interpolate2D(pt.Data2D, inputs[0], inputs[1]), nil
	case 3:
		if len(inputs) != 3 {
			return 0, fmt.Errorf("3D table requires 3 inputs, got %d", len(inputs))
		}
		return interpolate3D(pt.Data3D, inputs[0], inputs[1], inputs[2]), nil
	default:
		return 0, fmt.Errorf("unsupported table dimension: %d", pt.Dimension)
	}
}

// interpolate1D performs 1D linear interpolation
func interpolate1D(t *Table1D, x float64) float64 {
	n := len(t.Indices)
	if n == 0 {
		return 0
	}
	
	// Find bracketing indices
	if x <= t.Indices[0] {
		return t.Values[0]
	}
	if x >= t.Indices[n-1] {
		return t.Values[n-1]
	}
	
	for i := 0; i < n-1; i++ {
		if x >= t.Indices[i] && x <= t.Indices[i+1] {
			// Linear interpolation
			frac := (x - t.Indices[i]) / (t.Indices[i+1] - t.Indices[i])
			return t.Values[i] + frac*(t.Values[i+1]-t.Values[i])
		}
	}
	
	return t.Values[n-1]
}

// interpolate2D performs 2D bilinear interpolation
func interpolate2D(t *Table2D, row, col float64) float64 {
	if t == nil || len(t.Data) == 0 {
		return 0
	}
	
	// Handle empty data
	if len(t.RowIndices) == 0 || len(t.ColIndices) == 0 || len(t.Data) == 0 {
		return 0
	}
	
	// Check if any data rows are empty
	for _, row := range t.Data {
		if len(row) == 0 {
			return 0
		}
	}
	
	// Find row indices
	rowIdx1, rowIdx2, rowFrac := findIndices(t.RowIndices, row)
	// Find column indices
	colIdx1, colIdx2, colFrac := findIndices(t.ColIndices, col)
	
	// Ensure indices are valid
	if rowIdx1 >= len(t.Data) || rowIdx2 >= len(t.Data) {
		return 0
	}
	if len(t.Data[0]) == 0 || colIdx1 >= len(t.Data[0]) || colIdx2 >= len(t.Data[0]) {
		return 0
	}
	
	// Handle single column case
	if len(t.ColIndices) == 1 || colIdx1 == colIdx2 {
		if len(t.RowIndices) == 1 || rowIdx1 == rowIdx2 {
			// Single point
			return t.Data[rowIdx1][colIdx1]
		}
		// Single column, interpolate along rows
		return t.Data[rowIdx1][colIdx1] + rowFrac*(t.Data[rowIdx2][colIdx1]-t.Data[rowIdx1][colIdx1])
	}
	
	// Handle single row case
	if len(t.RowIndices) == 1 || rowIdx1 == rowIdx2 {
		// Single row, interpolate along columns
		return t.Data[rowIdx1][colIdx1] + colFrac*(t.Data[rowIdx1][colIdx2]-t.Data[rowIdx1][colIdx1])
	}
	
	// Bilinear interpolation
	v11 := t.Data[rowIdx1][colIdx1]
	v12 := t.Data[rowIdx1][colIdx2]
	v21 := t.Data[rowIdx2][colIdx1]
	v22 := t.Data[rowIdx2][colIdx2]
	
	v1 := v11 + colFrac*(v12-v11)
	v2 := v21 + colFrac*(v22-v21)
	
	return v1 + rowFrac*(v2-v1)
}

// interpolate3D performs 3D trilinear interpolation
func interpolate3D(tables []*Table2D, row, col, table float64) float64 {
	if len(tables) == 0 {
		return 0
	}
	
	// Find table indices
	tableIdx1, tableIdx2 := 0, 0
	tableFrac := 0.0
	
	for i := 0; i < len(tables)-1; i++ {
		if table >= tables[i].Breakpoint && table <= tables[i+1].Breakpoint {
			tableIdx1 = i
			tableIdx2 = i + 1
			if tables[i+1].Breakpoint != tables[i].Breakpoint {
				tableFrac = (table - tables[i].Breakpoint) / 
				           (tables[i+1].Breakpoint - tables[i].Breakpoint)
			}
			break
		}
	}
	
	// If out of bounds, use nearest table
	if table < tables[0].Breakpoint {
		return interpolate2D(tables[0], row, col)
	}
	if table > tables[len(tables)-1].Breakpoint {
		return interpolate2D(tables[len(tables)-1], row, col)
	}
	
	// Interpolate between two 2D tables
	v1 := interpolate2D(tables[tableIdx1], row, col)
	v2 := interpolate2D(tables[tableIdx2], row, col)
	
	return v1 + tableFrac*(v2-v1)
}

// findIndices finds bracketing indices for interpolation
func findIndices(indices []float64, value float64) (int, int, float64) {
	n := len(indices)
	if n == 0 {
		return 0, 0, 0
	}
	
	// Check bounds
	if value <= indices[0] {
		return 0, 0, 0
	}
	if value >= indices[n-1] {
		return n-1, n-1, 0
	}
	
	// Find bracketing indices
	for i := 0; i < n-1; i++ {
		if value >= indices[i] && value <= indices[i+1] {
			frac := 0.0
			if indices[i+1] != indices[i] {
				frac = (value - indices[i]) / (indices[i+1] - indices[i])
			}
			return i, i+1, frac
		}
	}
	
	return n-1, n-1, 0
}

// EvaluateFunction evaluates a mathematical function
func EvaluateFunction(f *Function, properties map[string]float64) (float64, error) {
	if f == nil {
		return 0, fmt.Errorf("function is nil")
	}
	
	if f.Product != nil {
		return evaluateOperation(f.Product, "product", properties)
	}
	if f.Sum != nil {
		return evaluateOperation(f.Sum, "sum", properties)
	}
	if f.Difference != nil {
		return evaluateOperation(f.Difference, "difference", properties)
	}
	if f.Quotient != nil {
		return evaluateOperation(f.Quotient, "quotient", properties)
	}
	if f.Pow != nil {
		return evaluateOperation(f.Pow, "pow", properties)
	}
	if f.Abs != nil {
		return evaluateOperation(f.Abs, "abs", properties)
	}
	if f.Sin != nil {
		return evaluateOperation(f.Sin, "sin", properties)
	}
	if f.Cos != nil {
		return evaluateOperation(f.Cos, "cos", properties)
	}
	if f.Tan != nil {
		return evaluateOperation(f.Tan, "tan", properties)
	}
	if f.Asin != nil {
		return evaluateOperation(f.Asin, "asin", properties)
	}
	if f.Acos != nil {
		return evaluateOperation(f.Acos, "acos", properties)
	}
	if f.Atan != nil {
		return evaluateOperation(f.Atan, "atan", properties)
	}
	if f.Table != nil {
		// Table evaluation would require current values of independent variables
		// This is a simplified version
		pt, err := ParseTable(f.Table)
		if err != nil {
			return 0, err
		}
		
		// Get input values from properties
		inputs := make([]float64, len(pt.IndependentVars))
		for i, varName := range pt.IndependentVars {
			inputs[i] = properties[varName]
		}
		
		return InterpolateTable(pt, inputs...)
	}
	
	return 0, fmt.Errorf("no valid operation in function")
}

// evaluateOperation evaluates a mathematical operation
func evaluateOperation(op *Operation, opType string, properties map[string]float64) (float64, error) {
	values := make([]float64, 0)
	
	// Collect values from properties
	for _, prop := range op.Property {
		if val, ok := properties[prop]; ok {
			values = append(values, val)
		}
	}
	
	// Add literal values
	values = append(values, op.Value...)
	
	// Evaluate nested operations
	if op.Product != nil {
		val, err := evaluateOperation(op.Product, "product", properties)
		if err == nil {
			values = append(values, val)
		}
	}
	if op.Sum != nil {
		val, err := evaluateOperation(op.Sum, "sum", properties)
		if err == nil {
			values = append(values, val)
		}
	}
	if op.Difference != nil {
		val, err := evaluateOperation(op.Difference, "difference", properties)
		if err == nil {
			values = append(values, val)
		}
	}
	if op.Quotient != nil {
		val, err := evaluateOperation(op.Quotient, "quotient", properties)
		if err == nil {
			values = append(values, val)
		}
	}
	
	// Evaluate table if present
	if op.Table != nil {
		pt, err := ParseTable(op.Table)
		if err == nil {
			inputs := make([]float64, len(pt.IndependentVars))
			for i, varName := range pt.IndependentVars {
				inputs[i] = properties[varName]
			}
			val, err := InterpolateTable(pt, inputs...)
			if err == nil {
				values = append(values, val)
			}
		}
	}
	
	// Perform the operation
	if len(values) == 0 {
		return 0, fmt.Errorf("no values for operation")
	}
	
	return performOperation(opType, values), nil
}

// performOperation performs the actual mathematical operation
func performOperation(opType string, values []float64) float64 {
	if len(values) == 0 {
		return 0
	}
	
	switch opType {
	case "product":
		result := 1.0
		for _, v := range values {
			result *= v
		}
		return result
	case "sum":
		result := 0.0
		for _, v := range values {
			result += v
		}
		return result
	case "difference":
		if len(values) < 2 {
			return values[0]
		}
		result := values[0]
		for i := 1; i < len(values); i++ {
			result -= values[i]
		}
		return result
	case "quotient":
		if len(values) < 2 {
			return values[0]
		}
		result := values[0]
		for i := 1; i < len(values); i++ {
			if values[i] != 0 {
				result /= values[i]
			}
		}
		return result
	case "pow":
		if len(values) < 2 {
			return values[0]
		}
		return pow(values[0], values[1])
	case "abs":
		return abs(values[0])
	case "sin":
		return sin(values[0])
	case "cos":
		return cos(values[0])
	case "tan":
		return tan(values[0])
	case "asin":
		return asin(values[0])
	case "acos":
		return acos(values[0])
	case "atan":
		return atan(values[0])
	default:
		return values[0]
	}
}

// Math helper functions (simplified versions)
func pow(x, y float64) float64 {
	// In production, use math.Pow
	result := 1.0
	for i := 0; i < int(y); i++ {
		result *= x
	}
	return result
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

func sin(x float64) float64 {
	// In production, use math.Sin
	// This is a simplified Taylor series approximation
	x = fmod(x, 2*3.14159265359)
	result := x
	term := x
	for i := 1; i <= 10; i++ {
		term *= -x * x / float64((2*i)*(2*i+1))
		result += term
	}
	return result
}

func cos(x float64) float64 {
	// In production, use math.Cos
	return sin(x + 3.14159265359/2)
}

func tan(x float64) float64 {
	// In production, use math.Tan
	c := cos(x)
	if c != 0 {
		return sin(x) / c
	}
	return 0
}

func asin(x float64) float64 {
	// In production, use math.Asin
	// Simplified approximation
	if x < -1 || x > 1 {
		return 0
	}
	return x + (x*x*x)/6 + (3*x*x*x*x*x)/40
}

func acos(x float64) float64 {
	// In production, use math.Acos
	return 3.14159265359/2 - asin(x)
}

func atan(x float64) float64 {
	// In production, use math.Atan
	// Simplified approximation
	return x - (x*x*x)/3 + (x*x*x*x*x)/5
}

func fmod(x, y float64) float64 {
	if y == 0 {
		return x
	}
	return x - y*float64(int(x/y))
}

// ExtractAllValues extracts all values from the configuration
func ExtractAllValues(config *JSBSimConfig) map[string]interface{} {
	values := make(map[string]interface{})
	
	// Extract header information
	if config.Header != nil {
		values["header.author"] = config.Header.Author
		values["header.creation_date"] = config.Header.FileCreationDate
		values["header.description"] = config.Header.Description
		values["header.version"] = config.Header.Version
		
		refs := make([]map[string]string, len(config.Header.References))
		for i, ref := range config.Header.References {
			refs[i] = map[string]string{
				"refID":  ref.RefID,
				"author": ref.Author,
				"title":  ref.Title,
				"date":   ref.Date,
			}
		}
		values["header.references"] = refs
	}
	
	// Extract metrics
	if config.Metrics != nil {
		if config.Metrics.WingArea != nil {
			values["metrics.wing_area"] = config.Metrics.WingArea.Value
		}
		if config.Metrics.WingSpan != nil {
			values["metrics.wing_span"] = config.Metrics.WingSpan.Value
		}
		if config.Metrics.Chord != nil {
			values["metrics.chord"] = config.Metrics.Chord.Value
		}
		if config.Metrics.HTailArea != nil {
			values["metrics.htail_area"] = config.Metrics.HTailArea.Value
		}
		if config.Metrics.HTailArm != nil {
			values["metrics.htail_arm"] = config.Metrics.HTailArm.Value
		}
		if config.Metrics.VTailArea != nil {
			values["metrics.vtail_area"] = config.Metrics.VTailArea.Value
		}
		if config.Metrics.VTailArm != nil {
			values["metrics.vtail_arm"] = config.Metrics.VTailArm.Value
		}
		
		for _, loc := range config.Metrics.Location {
			key := fmt.Sprintf("metrics.location.%s", loc.Name)
			values[key] = map[string]float64{
				"x": loc.X,
				"y": loc.Y,
				"z": loc.Z,
			}
		}
	}
	
	// Extract mass balance
	if config.MassBalance != nil {
		if config.MassBalance.IXX != nil {
			values["mass_balance.ixx"] = config.MassBalance.IXX.Value
		}
		if config.MassBalance.IYY != nil {
			values["mass_balance.iyy"] = config.MassBalance.IYY.Value
		}
		if config.MassBalance.IZZ != nil {
			values["mass_balance.izz"] = config.MassBalance.IZZ.Value
		}
		if config.MassBalance.IXY != nil {
			values["mass_balance.ixy"] = config.MassBalance.IXY.Value
		}
		if config.MassBalance.IXZ != nil {
			values["mass_balance.ixz"] = config.MassBalance.IXZ.Value
		}
		if config.MassBalance.IYZ != nil {
			values["mass_balance.iyz"] = config.MassBalance.IYZ.Value
		}
		if config.MassBalance.EmptyMass != nil {
			values["mass_balance.empty_mass"] = config.MassBalance.EmptyMass.Value
		}
		
		if config.MassBalance.Location != nil {
			values["mass_balance.cg_location"] = map[string]float64{
				"x": config.MassBalance.Location.X,
				"y": config.MassBalance.Location.Y,
				"z": config.MassBalance.Location.Z,
			}
		}
		
		for i, pm := range config.MassBalance.PointMass {
			key := fmt.Sprintf("mass_balance.point_mass.%d", i)
			pmData := map[string]interface{}{
				"name": pm.Name,
			}
			if pm.Mass != nil {
				pmData["mass"] = pm.Mass.Value
			}
			if pm.Location != nil {
				pmData["location"] = map[string]float64{
					"x": pm.Location.X,
					"y": pm.Location.Y,
					"z": pm.Location.Z,
				}
			}
			values[key] = pmData
		}
	}
	
	// Extract ground reactions
	if config.GroundReactions != nil {
		for i, contact := range config.GroundReactions.Contact {
			key := fmt.Sprintf("ground_reactions.contact.%d", i)
			contactData := map[string]interface{}{
				"type":             contact.Type,
				"name":             contact.Name,
				"static_friction":  contact.StaticFriction,
				"dynamic_friction": contact.DynamicFriction,
				"rolling_friction": contact.RollingFriction,
				"brake_group":      contact.BrakeGroup,
				"retractable":      contact.Retractable,
			}
			
			if contact.Location != nil {
				contactData["location"] = map[string]float64{
					"x": contact.Location.X,
					"y": contact.Location.Y,
					"z": contact.Location.Z,
				}
			}
			
			if contact.Spring != nil {
				contactData["spring_constant"] = contact.Spring.Constant
			}
			
			if contact.Damper != nil {
				contactData["damper_constant"] = contact.Damper.Constant
			}
			
			if contact.SpringCoeff != nil {
				contactData["spring_constant"] = contact.SpringCoeff.Value
			}
			
			if contact.DampingCoeff != nil {
				contactData["damper_constant"] = contact.DampingCoeff.Value
			}
			
			if contact.MaxSteer != nil {
				contactData["max_steer"] = contact.MaxSteer.Value
			}
			
			values[key] = contactData
		}
	}
	
	// Extract propulsion
	if config.Propulsion != nil {
		for i, engine := range config.Propulsion.Engine {
			key := fmt.Sprintf("propulsion.engine.%d", i)
			engineData := map[string]interface{}{
				"file": engine.File,
				"name": engine.Name,
				"feed": engine.Feed,
			}
			
			if engine.Location != nil {
				engineData["location"] = map[string]float64{
					"x": engine.Location.X,
					"y": engine.Location.Y,
					"z": engine.Location.Z,
				}
			}
			
			if engine.Orient != nil {
				engineData["orientation"] = map[string]float64{
					"pitch": engine.Orient.Pitch,
					"roll":  engine.Orient.Roll,
					"yaw":   engine.Orient.Yaw,
				}
			}
			
			if engine.Thruster != nil {
				engineData["thruster"] = map[string]interface{}{
					"file": engine.Thruster.File,
					"name": engine.Thruster.Name,
				}
			}
			
			values[key] = engineData
		}
		
		for i, tank := range config.Propulsion.Tank {
			key := fmt.Sprintf("propulsion.tank.%d", i)
			tankData := map[string]interface{}{
				"type":        tank.Type,
				"number":      tank.Number,
				"temperature": tank.Temperature,
			}
			
			if tank.Location != nil {
				tankData["location"] = map[string]float64{
					"x": tank.Location.X,
					"y": tank.Location.Y,
					"z": tank.Location.Z,
				}
			}
			
			if tank.Capacity != nil {
				tankData["capacity"] = tank.Capacity.Value
			}
			
			if tank.Contents != nil {
				tankData["contents"] = tank.Contents.Value
			}
			
			values[key] = tankData
		}
	}
	
	// Extract flight control
	if config.FlightControl != nil {
		extractFlightControl(config.FlightControl, "flight_control", values)
	}
	
	// Extract autopilot (if separate from flight control)
	if config.Autopilot != nil {
		extractFlightControl(config.Autopilot, "autopilot", values)
	}
	
	// Extract aerodynamics
	if config.Aerodynamics != nil {
		if config.Aerodynamics.AlphaLimits != nil {
			values["aerodynamics.alpha_limits"] = map[string]float64{
				"min": config.Aerodynamics.AlphaLimits.Min,
				"max": config.Aerodynamics.AlphaLimits.Max,
			}
		}
		
		for _, axis := range config.Aerodynamics.Axis {
			for j, fn := range axis.Function {
				key := fmt.Sprintf("aerodynamics.axis.%s.function.%d", axis.Name, j)
				values[key] = extractFunctionData(fn)
			}
		}
		
		for i, fn := range config.Aerodynamics.Function {
			key := fmt.Sprintf("aerodynamics.function.%d", i)
			values[key] = extractFunctionData(fn)
		}
	}
	
	// Extract I/O
	if config.Input != nil {
		values["input"] = map[string]interface{}{
			"port":     config.Input.Port,
			"protocol": config.Input.Protocol,
		}
	}
	
	if config.Output != nil {
		values["output"] = map[string]interface{}{
			"name":     config.Output.Name,
			"type":     config.Output.Type,
			"port":     config.Output.Port,
			"protocol": config.Output.Protocol,
			"rate":     config.Output.Rate,
		}
	}
	
	return values
}

// extractFlightControl extracts flight control data
func extractFlightControl(fc *FlightControl, prefix string, values map[string]interface{}) {
	values[prefix+".name"] = fc.Name
	values[prefix+".properties"] = fc.Property
	
	for i, rg := range fc.RateGroup {
		key := fmt.Sprintf("%s.rate_group.%d", prefix, i)
		values[key] = map[string]interface{}{
			"name":    rg.Name,
			"rate_hz": rg.RateHz,
		}
	}
	
	for _, ch := range fc.Channel {
		chPrefix := fmt.Sprintf("%s.channel.%s", prefix, ch.Name)
		
		for j, comp := range ch.Component {
			key := fmt.Sprintf("%s.component.%d", chPrefix, j)
			compData := map[string]interface{}{
				"name":       comp.Name,
				"type":       comp.Type,
				"rate_group": comp.RateGroup,
				"input":      comp.Input,
				"output":     comp.Output,
				"gain":       comp.Gain,
			}
			
			if comp.Clipto != nil {
				compData["clip_min"] = comp.Clipto.Min
				compData["clip_max"] = comp.Clipto.Max
			}
			
			// Add filter coefficients if present
			if comp.C1 != 0 {
				compData["c1"] = comp.C1
			}
			if comp.C2 != 0 {
				compData["c2"] = comp.C2
			}
			if comp.C3 != 0 {
				compData["c3"] = comp.C3
			}
			if comp.C4 != 0 {
				compData["c4"] = comp.C4
			}
			if comp.C5 != 0 {
				compData["c5"] = comp.C5
			}
			if comp.C6 != 0 {
				compData["c6"] = comp.C6
			}
			
			if comp.Width != 0 {
				compData["width"] = comp.Width
			}
			
			if comp.Function != nil {
				compData["function"] = extractFunctionData(comp.Function)
			}
			
			values[key] = compData
		}
		
		for j, sensor := range ch.Sensor {
			key := fmt.Sprintf("%s.sensor.%d", chPrefix, j)
			sensorData := map[string]interface{}{
				"name":       sensor.Name,
				"rate_group": sensor.RateGroup,
				"input":      sensor.Input,
				"lag":        sensor.Lag,
				"drift_rate": sensor.DriftRate,
				"bias":       sensor.Bias,
			}
			
			if sensor.Noise != nil {
				sensorData["noise"] = map[string]interface{}{
					"variation": sensor.Noise.Variation,
					"value":     sensor.Noise.Value,
				}
			}
			
			if sensor.Quantization != nil {
				sensorData["quantization"] = map[string]interface{}{
					"bits": sensor.Quantization.Bits,
					"min":  sensor.Quantization.Min,
					"max":  sensor.Quantization.Max,
				}
			}
			
			values[key] = sensorData
		}
	}
}

// extractFunctionData extracts function data
func extractFunctionData(fn *Function) map[string]interface{} {
	data := map[string]interface{}{
		"name":        fn.Name,
		"description": fn.Description,
	}
	
	if fn.Table != nil {
		tableData := map[string]interface{}{
			"name": fn.Table.Name,
		}
		
		indVars := make([]map[string]string, len(fn.Table.IndependentVar))
		for i, iv := range fn.Table.IndependentVar {
			indVars[i] = map[string]string{
				"lookup": iv.Lookup,
				"value":  iv.Value,
			}
		}
		tableData["independent_vars"] = indVars
		
		// Parse table data
		if pt, err := ParseTable(fn.Table); err == nil {
			tableData["dimension"] = pt.Dimension
			if pt.Data1D != nil {
				tableData["data_1d"] = map[string]interface{}{
					"indices": pt.Data1D.Indices,
					"values":  pt.Data1D.Values,
				}
			}
			if pt.Data2D != nil {
				tableData["data_2d"] = map[string]interface{}{
					"row_indices": pt.Data2D.RowIndices,
					"col_indices": pt.Data2D.ColIndices,
					"data":        pt.Data2D.Data,
				}
			}
			if pt.Data3D != nil {
				tables3D := make([]map[string]interface{}, len(pt.Data3D))
				for i, t2d := range pt.Data3D {
					tables3D[i] = map[string]interface{}{
						"breakpoint":  t2d.Breakpoint,
						"row_indices": t2d.RowIndices,
						"col_indices": t2d.ColIndices,
						"data":        t2d.Data,
					}
				}
				tableData["data_3d"] = tables3D
			}
		}
		
		data["table"] = tableData
	}
	
	// Extract operation type
	if fn.Product != nil {
		data["operation"] = "product"
	} else if fn.Sum != nil {
		data["operation"] = "sum"
	} else if fn.Difference != nil {
		data["operation"] = "difference"
	} else if fn.Quotient != nil {
		data["operation"] = "quotient"
	} else if fn.Pow != nil {
		data["operation"] = "pow"
	} else if fn.Abs != nil {
		data["operation"] = "abs"
	} else if fn.Sin != nil {
		data["operation"] = "sin"
	} else if fn.Cos != nil {
		data["operation"] = "cos"
	} else if fn.Tan != nil {
		data["operation"] = "tan"
	} else if fn.Asin != nil {
		data["operation"] = "asin"
	} else if fn.Acos != nil {
		data["operation"] = "acos"
	} else if fn.Atan != nil {
		data["operation"] = "atan"
	}
	
	return data
}