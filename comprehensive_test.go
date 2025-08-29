package main

import (
	"fmt"
	"os"
	"strings"
	"testing"
	"time"
)

// Test ParseTable with various table types
func TestParseTable(t *testing.T) {
	t.Run("1D Table", func(t *testing.T) {
		table := &Table{
			Name: "test-1d",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
			},
			TableData: []*TableData{
				{Data: "-10.0  0.1\n0.0    0.0\n10.0   0.1"},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse 1D table: %v", err)
		}

		assertEqual(t, pt.Name, "test-1d")
		assertEqual(t, pt.Dimension, 1)
		assertEqual(t, len(pt.IndependentVars), 1)
		assertEqual(t, pt.IndependentVars[0], "aero/alpha-deg")
		assertEqual(t, pt.LookupTypes[0], "row")

		if pt.Data1D == nil {
			t.Fatal("Expected Data1D to be non-nil")
		}
		assertEqual(t, len(pt.Data1D.Indices), 3)
		assertEqual(t, len(pt.Data1D.Values), 3)
		assertEqual(t, pt.Data1D.Indices[0], -10.0)
		assertEqual(t, pt.Data1D.Values[1], 0.0)
		assertEqual(t, pt.Data1D.Values[2], 0.1)
	})

	t.Run("2D Table", func(t *testing.T) {
		table := &Table{
			Name: "test-2d",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
				{Lookup: "column", Value: "aero/mach"},
			},
			TableData: []*TableData{
				{Data: "        0.5    0.8    1.0\n-10.0   0.1    0.2    0.3\n0.0     0.0    0.1    0.2\n10.0    0.1    0.3    0.5"},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse 2D table: %v", err)
		}

		assertEqual(t, pt.Dimension, 2)
		assertEqual(t, len(pt.IndependentVars), 2)

		if pt.Data2D == nil {
			t.Fatal("Expected Data2D to be non-nil")
		}
		assertEqual(t, len(pt.Data2D.ColIndices), 3)
		assertEqual(t, len(pt.Data2D.RowIndices), 3)
		assertEqual(t, len(pt.Data2D.Data), 3)
		assertEqual(t, pt.Data2D.ColIndices[0], 0.5)
		assertEqual(t, pt.Data2D.Data[0][0], 0.1)
		assertEqual(t, pt.Data2D.Data[2][2], 0.5)
	})

	t.Run("3D Table", func(t *testing.T) {
		table := &Table{
			Name: "test-3d",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
				{Lookup: "column", Value: "aero/mach"},
				{Lookup: "table", Value: "atmosphere/altitude-ft"},
			},
			TableData: []*TableData{
				{
					Breakpoint: "0.0",
					Data:       "        0.5    0.8\n-10.0   0.1    0.2\n10.0    0.3    0.4",
				},
				{
					Breakpoint: "10000.0",
					Data:       "        0.5    0.8\n-10.0   0.15   0.25\n10.0    0.35   0.45",
				},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse 3D table: %v", err)
		}

		assertEqual(t, pt.Dimension, 3)
		assertEqual(t, len(pt.Data3D), 2)
		assertEqual(t, pt.Data3D[0].Breakpoint, 0.0)
		assertEqual(t, pt.Data3D[1].Breakpoint, 10000.0)
		assertEqual(t, len(pt.Data3D[0].Data), 2)
		assertEqual(t, pt.Data3D[0].Data[0][0], 0.1)
		assertEqual(t, pt.Data3D[1].Data[1][1], 0.45)
	})
}

// Test InterpolateTable with various scenarios
func TestInterpolateTable(t *testing.T) {
	t.Run("1D Interpolation", func(t *testing.T) {
		table1D := &ParsedTable{
			Dimension: 1,
			Data1D: &Table1D{
				Indices: []float64{-10.0, 0.0, 10.0, 20.0},
				Values:  []float64{0.1, 0.0, 0.1, 0.2},
			},
		}

		// Test exact match
		result, err := InterpolateTable(table1D, 0.0)
		if err != nil {
			t.Fatalf("Error interpolating: %v", err)
		}
		assertEqual(t, result, 0.0)

		// Test interpolation between points
		result, err = InterpolateTable(table1D, 5.0)
		if err != nil {
			t.Fatalf("Error interpolating: %v", err)
		}
		assertEqual(t, result, 0.05) // halfway between 0.0 and 0.1

		// Test extrapolation (below range)
		result, err = InterpolateTable(table1D, -20.0)
		if err != nil {
			t.Fatalf("Error interpolating: %v", err)
		}
		assertEqual(t, result, 0.1) // should return first value

		// Test extrapolation (above range)
		result, err = InterpolateTable(table1D, 30.0)
		if err != nil {
			t.Fatalf("Error interpolating: %v", err)
		}
		assertEqual(t, result, 0.2) // should return last value
	})

	t.Run("2D Interpolation", func(t *testing.T) {
		table2D := &ParsedTable{
			Dimension: 2,
			Data2D: &Table2D{
				RowIndices: []float64{0.0, 10.0},
				ColIndices: []float64{0.5, 1.0},
				Data: [][]float64{
					{0.1, 0.2},
					{0.3, 0.4},
				},
			},
		}

		// Test exact corner
		result, err := InterpolateTable(table2D, 0.0, 0.5)
		if err != nil {
			t.Fatalf("Error interpolating 2D: %v", err)
		}
		assertEqual(t, result, 0.1)

		// Test center interpolation
		result, err = InterpolateTable(table2D, 5.0, 0.75)
		if err != nil {
			t.Fatalf("Error interpolating 2D: %v", err)
		}
		assertEqual(t, result, 0.25) // bilinear interpolation result
	})

	t.Run("Wrong Input Count", func(t *testing.T) {
		table1D := &ParsedTable{
			Dimension: 1,
			Data1D: &Table1D{
				Indices: []float64{0.0, 1.0},
				Values:  []float64{0.0, 1.0},
			},
		}

		// Should fail with wrong number of inputs
		_, err := InterpolateTable(table1D, 0.0, 1.0)
		if err == nil {
			t.Error("Expected error for wrong input count, got nil")
		}
	})
}

// Test EvaluateFunction with different operations
func TestEvaluateFunction(t *testing.T) {
	properties := map[string]float64{
		"test/prop1": 2.0,
		"test/prop2": 3.0,
		"test/alpha": 5.0,
	}

	t.Run("Product Operation", func(t *testing.T) {
		fn := &Function{
			Name: "test-product",
			Product: &Operation{
				Property: []string{"test/prop1", "test/prop2"},
				Value:    []float64{4.0},
			},
		}

		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("Error evaluating product function: %v", err)
		}
		assertEqual(t, result, 24.0) // 2.0 * 3.0 * 4.0
	})

	t.Run("Sum Operation", func(t *testing.T) {
		fn := &Function{
			Name: "test-sum",
			Sum: &Operation{
				Property: []string{"test/prop1", "test/prop2"},
				Value:    []float64{1.0},
			},
		}

		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("Error evaluating sum function: %v", err)
		}
		assertEqual(t, result, 6.0) // 2.0 + 3.0 + 1.0
	})

	t.Run("Difference Operation", func(t *testing.T) {
		fn := &Function{
			Name: "test-difference",
			Difference: &Operation{
				Property: []string{"test/prop1"},
				Value:    []float64{1.0},
			},
		}

		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("Error evaluating difference function: %v", err)
		}
		assertEqual(t, result, 1.0) // 2.0 - 1.0
	})

	t.Run("Nested Operations", func(t *testing.T) {
		fn := &Function{
			Name: "test-nested",
			Product: &Operation{
				Property: []string{"test/prop1"},
				Sum: &Operation{
					Property: []string{"test/prop2"},
					Value:    []float64{2.0},
				},
			},
		}

		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("Error evaluating nested function: %v", err)
		}
		assertEqual(t, result, 10.0) // 2.0 * (3.0 + 2.0)
	})
}

// Test math helper functions
func TestMathHelpers(t *testing.T) {
	t.Run("Power Function", func(t *testing.T) {
		assertEqual(t, pow(2.0, 3.0), 8.0)
		assertEqual(t, pow(5.0, 2.0), 25.0)
		assertEqual(t, pow(10.0, 0.0), 1.0)
	})

	t.Run("Absolute Value", func(t *testing.T) {
		assertEqual(t, abs(-5.0), 5.0)
		assertEqual(t, abs(5.0), 5.0)
		assertEqual(t, abs(0.0), 0.0)
	})

	t.Run("Trigonometric Functions", func(t *testing.T) {
		// Test basic values (approximate due to simplified implementation)
		assertApproxEqual(t, sin(0.0), 0.0, 0.01)
		assertApproxEqual(t, cos(0.0), 1.0, 0.01)
		assertApproxEqual(t, tan(0.0), 0.0, 0.01)
		
		// Test small angles
		assertApproxEqual(t, sin(0.1), 0.1, 0.01)
		assertApproxEqual(t, asin(0.1), 0.1, 0.02)
	})
}

// Test with actual P-51D file data
func TestRealP51DData(t *testing.T) {
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		t.Fatalf("Failed to open P-51D XML: %v", err)
	}
	defer file.Close()

	config, err := ParseJSBSimConfig(file)
	if err != nil {
		t.Fatalf("Failed to parse P-51D XML: %v", err)
	}

	t.Run("Aerodynamics Tables", func(t *testing.T) {
		if config.Aerodynamics == nil {
			t.Fatal("No aerodynamics section found")
		}

		tableCount := 0
		functionsWithTables := []string{}

		// Check standalone functions
		for _, fn := range config.Aerodynamics.Function {
			if fn.Table != nil {
				tableCount++
				functionsWithTables = append(functionsWithTables, fn.Name)
				
				// Test parsing
				pt, err := ParseTable(fn.Table)
				if err != nil {
					t.Errorf("Failed to parse table in function %s: %v", fn.Name, err)
					continue
				}
				
				// Test interpolation
				if pt.Dimension == 1 {
					_, err = InterpolateTable(pt, 0.1)
					if err != nil {
						t.Errorf("Failed to interpolate 1D table in %s: %v", fn.Name, err)
					}
				}
			}
		}

		// Check axis functions
		for _, axis := range config.Aerodynamics.Axis {
			for _, fn := range axis.Function {
				// Check direct tables
				if fn.Table != nil {
					tableCount++
					functionsWithTables = append(functionsWithTables, fn.Name)
				}
				
				// Check tables in operations
				if fn.Product != nil && fn.Product.Table != nil {
					tableCount++
					functionsWithTables = append(functionsWithTables, fn.Name+" (product)")
					
					// Test parsing and interpolation
					pt, err := ParseTable(fn.Product.Table)
					if err != nil {
						t.Errorf("Failed to parse product table in %s: %v", fn.Name, err)
						continue
					}
					
					// Test appropriate interpolation based on dimension
					if pt.Dimension == 1 && pt.Data1D != nil && len(pt.Data1D.Indices) > 0 {
						// Use a value within the table range
						minIdx := pt.Data1D.Indices[0]
						maxIdx := pt.Data1D.Indices[len(pt.Data1D.Indices)-1]
						testVal := (minIdx + maxIdx) / 2.0
						_, err = InterpolateTable(pt, testVal)
						if err != nil {
							t.Errorf("Failed to interpolate 1D product table in %s: %v", fn.Name, err)
						}
					} else if pt.Dimension == 2 && pt.Data2D != nil && 
						len(pt.Data2D.RowIndices) > 0 && len(pt.Data2D.ColIndices) > 0 {
						// Use values within the table range
						minRow := pt.Data2D.RowIndices[0]
						maxRow := pt.Data2D.RowIndices[len(pt.Data2D.RowIndices)-1]
						minCol := pt.Data2D.ColIndices[0]
						maxCol := pt.Data2D.ColIndices[len(pt.Data2D.ColIndices)-1]
						testRow := (minRow + maxRow) / 2.0
						testCol := (minCol + maxCol) / 2.0
						_, err = InterpolateTable(pt, testRow, testCol)
						if err != nil {
							t.Errorf("Failed to interpolate 2D product table in %s: %v", fn.Name, err)
						}
					}
				}
			}
		}

		t.Logf("Found %d tables in functions: %v", tableCount, functionsWithTables)
		
		if tableCount < 10 {
			t.Errorf("Expected at least 10 tables in P-51D data, found %d", tableCount)
		}
	})

	t.Run("Flight Control Tables", func(t *testing.T) {
		if config.FlightControl == nil {
			t.Fatal("No flight control section found")
		}

		fcTableCount := 0
		for _, channel := range config.FlightControl.Channel {
			for _, component := range channel.Component {
				if component.Function != nil && component.Function.Table != nil {
					fcTableCount++
					
					// Test table parsing
					pt, err := ParseTable(component.Function.Table)
					if err != nil {
						t.Errorf("Failed to parse FC table in component %s: %v", component.Name, err)
						continue
					}
					
					// Test interpolation based on dimension
					switch pt.Dimension {
					case 1:
						_, err = InterpolateTable(pt, 10.0)
					case 2:
						_, err = InterpolateTable(pt, 10.0, 50.0)
					case 3:
						_, err = InterpolateTable(pt, 10.0, 50.0, 20.0)
					}
					
					if err != nil {
						t.Errorf("Failed to interpolate FC table in %s: %v", component.Name, err)
					}
				}
			}
		}

		t.Logf("Found %d tables in flight control", fcTableCount)
	})

	t.Run("Complete Value Extraction", func(t *testing.T) {
		values := ExtractAllValues(config)
		
		// Test that we have substantial data
		if len(values) < 50 {
			t.Errorf("Expected at least 50 extracted values, got %d", len(values))
		}

		// Test key sections are present
		expectedKeys := []string{
			"header.author",
			"metrics.wing_area",
			"mass_balance.ixx",
			"propulsion.engine.0",
			"flight_control.name",
		}

		for _, key := range expectedKeys {
			if _, exists := values[key]; !exists {
				t.Errorf("Expected key %s not found in extracted values", key)
			}
		}

		// Test numeric values are correct types
		if wingArea, ok := values["metrics.wing_area"].(float64); !ok || wingArea != 235.0 {
			t.Errorf("Expected wing area 235.0, got %v (%T)", values["metrics.wing_area"], values["metrics.wing_area"])
		}

		// Test nested structures
		if engine, ok := values["propulsion.engine.0"].(map[string]interface{}); ok {
			if engineFile, ok := engine["file"].(string); !ok || engineFile != "Packard-V-1650-7" {
				t.Errorf("Expected engine file 'Packard-V-1650-7', got %v", engine["file"])
			}
		} else {
			t.Errorf("Expected engine to be map[string]interface{}, got %T", values["propulsion.engine.0"])
		}
	})
}

// Test error handling and edge cases
func TestErrorHandling(t *testing.T) {
	t.Run("Empty Table Data", func(t *testing.T) {
		table := &Table{
			Name: "empty",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "test"},
			},
			TableData: []*TableData{
				{Data: ""},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("ParseTable should handle empty data gracefully: %v", err)
		}
		
		if pt.Data1D == nil || len(pt.Data1D.Indices) != 0 {
			t.Error("Expected empty 1D table data")
		}
	})

	t.Run("Invalid Table Dimension", func(t *testing.T) {
		pt := &ParsedTable{Dimension: 5}
		_, err := InterpolateTable(pt, 1.0)
		if err == nil {
			t.Error("Expected error for invalid table dimension")
		}
	})

	t.Run("Nil Function", func(t *testing.T) {
		_, err := EvaluateFunction(nil, map[string]float64{})
		if err == nil {
			t.Error("Expected error for nil function")
		}
	})

	t.Run("Missing Properties", func(t *testing.T) {
		fn := &Function{
			Product: &Operation{
				Property: []string{"nonexistent/prop"},
			},
		}

		result, err := EvaluateFunction(fn, map[string]float64{})
		if err != nil {
			// This is expected behavior - when no values can be found, it should error
			// Let's test with a function that has some valid values
			fn2 := &Function{
				Product: &Operation{
					Property: []string{"nonexistent/prop"},
					Value:    []float64{2.0}, // At least one value
				},
			}
			
			result2, err2 := EvaluateFunction(fn2, map[string]float64{})
			if err2 != nil {
				t.Fatalf("Should handle missing properties with fallback values: %v", err2)
			}
			assertEqual(t, result2, 2.0) // Should use the literal value
		} else {
			assertEqual(t, result, 1.0) // Product with no values should be 1.0
		}
	})
}

// Benchmark tests
func BenchmarkParseTable1D(b *testing.B) {
	table := &Table{
		Name: "bench-1d",
		IndependentVar: []*IndependentVar{
			{Lookup: "row", Value: "test"},
		},
		TableData: []*TableData{
			{Data: "-10.0 0.1\n-5.0 0.05\n0.0 0.0\n5.0 0.05\n10.0 0.1"},
		},
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, err := ParseTable(table)
		if err != nil {
			b.Fatalf("Parse error: %v", err)
		}
	}
}

func BenchmarkInterpolate1D(b *testing.B) {
	pt := &ParsedTable{
		Dimension: 1,
		Data1D: &Table1D{
			Indices: []float64{-10.0, -5.0, 0.0, 5.0, 10.0},
			Values:  []float64{0.1, 0.05, 0.0, 0.05, 0.1},
		},
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, err := InterpolateTable(pt, 2.5)
		if err != nil {
			b.Fatalf("Interpolation error: %v", err)
		}
	}
}

func BenchmarkEvaluateFunction(b *testing.B) {
	fn := &Function{
		Product: &Operation{
			Property: []string{"test/prop1", "test/prop2"},
			Value:    []float64{2.0},
		},
	}
	
	properties := map[string]float64{
		"test/prop1": 3.0,
		"test/prop2": 4.0,
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, err := EvaluateFunction(fn, properties)
		if err != nil {
			b.Fatalf("Evaluation error: %v", err)
		}
	}
}

func BenchmarkFullP51DParsing(b *testing.B) {
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		b.StopTimer()
		file, err := os.Open("aircraft/p51d-jsbsim.xml")
		if err != nil {
			b.Fatalf("Failed to open file: %v", err)
		}
		b.StartTimer()

		config, err := ParseJSBSimConfig(file)
		if err != nil {
			file.Close()
			b.Fatalf("Parse error: %v", err)
		}

		_ = ExtractAllValues(config)
		file.Close()
	}
}

// Helper function for approximate equality
func assertApproxEqual(t *testing.T, actual, expected, tolerance float64) {
	t.Helper()
	if abs(actual-expected) > tolerance {
		t.Errorf("Expected %f ± %f, got %f", expected, tolerance, actual)
	}
}

// Test comprehensive table features
func TestTableFeatures(t *testing.T) {
	t.Run("Complex 2D Table with Many Points", func(t *testing.T) {
		// Create a larger 2D table
		data := "     0.0   0.2   0.4   0.6   0.8   1.0\n"
		data += "-10  0.1   0.12  0.15  0.19  0.24  0.30\n"
		data += "-5   0.05  0.06  0.08  0.11  0.15  0.20\n"
		data += "0    0.0   0.0   0.0   0.02  0.05  0.10\n"
		data += "5    0.05  0.06  0.08  0.11  0.15  0.20\n"
		data += "10   0.1   0.12  0.15  0.19  0.24  0.30\n"

		table := &Table{
			Name: "complex-2d",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
				{Lookup: "column", Value: "velocities/mach"},
			},
			TableData: []*TableData{
				{Data: data},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse complex 2D table: %v", err)
		}

		// Test various interpolation points
		testCases := []struct {
			alpha, mach, expected float64
			description           string
		}{
			{0.0, 0.0, 0.0, "origin"},
			{-5.0, 0.5, 0.095, "negative alpha, mid mach"},
			{2.5, 0.3, 0.044, "interpolated center"},
			{15.0, 1.2, 0.30, "extrapolated beyond range"},
		}

		for _, tc := range testCases {
			result, err := InterpolateTable(pt, tc.alpha, tc.mach)
			if err != nil {
				t.Errorf("Failed to interpolate %s: %v", tc.description, err)
				continue
			}
			assertApproxEqual(t, result, tc.expected, 0.01)
		}
	})

	t.Run("3D Table Interpolation", func(t *testing.T) {
		table := &Table{
			Name: "test-3d-detailed",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
				{Lookup: "column", Value: "velocities/mach"},
				{Lookup: "table", Value: "atmosphere/altitude-ft"},
			},
			TableData: []*TableData{
				{
					Breakpoint: "0.0",
					Data:       "     0.5   1.0\n-10  0.1   0.2\n10   0.3   0.4",
				},
				{
					Breakpoint: "10000.0",
					Data:       "     0.5   1.0\n-10  0.15  0.25\n10   0.35  0.45",
				},
				{
					Breakpoint: "20000.0",
					Data:       "     0.5   1.0\n-10  0.2   0.3\n10   0.4   0.5",
				},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse 3D table: %v", err)
		}

		// Test 3D interpolation
		result, err := InterpolateTable(pt, 0.0, 0.75, 5000.0)
		if err != nil {
			t.Fatalf("Failed to interpolate 3D: %v", err)
		}

		// Should interpolate between sea level and 10k ft, center alpha, 3/4 mach
		assertApproxEqual(t, result, 0.275, 0.01)
	})
}

// Test unit conversion functions
func TestUnitConversions(t *testing.T) {
	t.Run("Length Conversions", func(t *testing.T) {
		// Test meter to feet
		result := convertToStandardUnit(1.0, "M", "length")
		assertApproxEqual(t, result, M_TO_FT, 0.001)

		// Test inches to feet
		result = convertToStandardUnit(12.0, "IN", "length")
		assertApproxEqual(t, result, 1.0, 0.001)
	})

	t.Run("Area Conversions", func(t *testing.T) {
		result := convertToStandardUnit(1.0, "M2", "area")
		assertApproxEqual(t, result, M2_TO_FT2, 0.01)
	})

	t.Run("Mass Conversions", func(t *testing.T) {
		result := convertToStandardUnit(1.0, "KG", "mass")
		assertApproxEqual(t, result, KG_TO_LB, 0.01)
	})

	t.Run("Unknown Unit", func(t *testing.T) {
		result := convertToStandardUnit(5.0, "UNKNOWN", "length")
		assertEqual(t, result, 5.0) // Should return unchanged
	})
}

// Test specific P-51D aerodynamic functions
func TestP51DAerodynamics(t *testing.T) {
	file, err := os.Open("aircraft/p51d-jsbsim.xml")
	if err != nil {
		t.Skip("P-51D file not available")
	}
	defer file.Close()

	config, err := ParseJSBSimConfig(file)
	if err != nil {
		t.Fatalf("Failed to parse P-51D: %v", err)
	}

	t.Run("Drag Coefficients", func(t *testing.T) {
		dragAxis := findAxis(config.Aerodynamics.Axis, "DRAG")
		if dragAxis == nil {
			t.Fatal("DRAG axis not found")
		}

		// Test CDo function (base drag)
		cdoFunc := findFunction(dragAxis.Function, "aero/coefficient/CDo")
		if cdoFunc == nil {
			t.Fatal("CDo function not found")
		}

		if cdoFunc.Product != nil && cdoFunc.Product.Table != nil {
			pt, err := ParseTable(cdoFunc.Product.Table)
			if err != nil {
				t.Fatalf("Failed to parse CDo table: %v", err)
			}

			// Test at various alpha values
			alphas := []float64{-10.0, 0.0, 5.0, 10.0}
			for _, alpha := range alphas {
				result, err := InterpolateTable(pt, alpha)
				if err != nil {
					t.Errorf("Failed to interpolate CDo at alpha %f: %v", alpha, err)
				} else if result < 0 {
					t.Errorf("CDo should be positive, got %f at alpha %f", result, alpha)
				}
			}
		}
	})

	t.Run("Lift Coefficients", func(t *testing.T) {
		liftAxis := findAxis(config.Aerodynamics.Axis, "LIFT")
		if liftAxis == nil {
			t.Fatal("LIFT axis not found")
		}

		// Test CLalpha function
		clAlphaFunc := findFunction(liftAxis.Function, "aero/coefficient/CLalpha")
		if clAlphaFunc != nil && clAlphaFunc.Product != nil && clAlphaFunc.Product.Table != nil {
			pt, err := ParseTable(clAlphaFunc.Product.Table)
			if err != nil {
				t.Fatalf("Failed to parse CLalpha table: %v", err)
			}

			// Test lift curve slope behavior based on table dimension
			if pt.Dimension == 1 {
				cl0, err := InterpolateTable(pt, 0.0)
				if err != nil {
					t.Fatalf("Failed to get CL at alpha=0: %v", err)
				}

				cl5, err := InterpolateTable(pt, 5.0)
				if err != nil {
					t.Fatalf("Failed to get CL at alpha=5: %v", err)
				}

				// CL should increase with positive alpha
				if cl5 <= cl0 {
					t.Errorf("Expected CL to increase with alpha: CL(0)=%f, CL(5)=%f", cl0, cl5)
				}
			} else if pt.Dimension == 2 && pt.Data2D != nil && len(pt.Data2D.ColIndices) > 0 {
				// For 2D table, use middle column value
				midCol := (pt.Data2D.ColIndices[0] + pt.Data2D.ColIndices[len(pt.Data2D.ColIndices)-1]) / 2.0
				
				cl0, err := InterpolateTable(pt, 0.0, midCol)
				if err != nil {
					t.Fatalf("Failed to get CL at alpha=0: %v", err)
				}

				cl5, err := InterpolateTable(pt, 5.0, midCol)
				if err != nil {
					t.Fatalf("Failed to get CL at alpha=5: %v", err)
				}

				// CL should increase with positive alpha
				if cl5 <= cl0 {
					t.Errorf("Expected CL to increase with alpha: CL(0)=%f, CL(5)=%f", cl0, cl5)
				}
			}
		}
	})
}

// Helper functions
func findAxis(axes []*Axis, name string) *Axis {
	for _, axis := range axes {
		if axis.Name == name {
			return axis
		}
	}
	return nil
}

func findFunction(functions []*Function, name string) *Function {
	for _, fn := range functions {
		if fn.Name == name {
			return fn
		}
	}
	return nil
}

// Test performance with large datasets
func TestPerformanceWithLargeData(t *testing.T) {
	if testing.Short() {
		t.Skip("Skipping performance test in short mode")
	}

	t.Run("Large 1D Table", func(t *testing.T) {
		// Create a large 1D table
		var data strings.Builder
		for i := 0; i < 1000; i++ {
			alpha := float64(i-500) * 0.1 // -50 to 50 degrees
			cl := 0.1 * alpha             // Simple linear relationship
			data.WriteString(fmt.Sprintf("%.1f  %.6f\n", alpha, cl))
		}

		table := &Table{
			Name: "large-1d",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
			},
			TableData: []*TableData{
				{Data: data.String()},
			},
		}

		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse large table: %v", err)
		}

		assertEqual(t, len(pt.Data1D.Indices), 1000)

		// Test interpolation performance
		start := time.Now()
		for i := 0; i < 100; i++ {
			alpha := float64(i-50) * 0.33
			_, err := InterpolateTable(pt, alpha)
			if err != nil {
				t.Errorf("Interpolation failed at alpha %f: %v", alpha, err)
			}
		}
		duration := time.Since(start)

		t.Logf("100 interpolations on 1000-point table took %v", duration)
		if duration > time.Millisecond*10 {
			t.Errorf("Interpolation too slow: %v", duration)
		}
	})
}
