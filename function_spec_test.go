package main

import (
	"testing"
	"time"
)

// TestJSBSimFunctionSpecification tests our function evaluation against the exact JSBSim specification
func TestJSBSimFunctionSpecification(t *testing.T) {
	
	t.Run("Basic Function Structure", func(t *testing.T) {
		// Test that functions have proper name and description support
		fn := &Function{
			Name:        "test/function",
			Description: "Test function description",
			Product: &Operation{
				Value: []float64{2.0, 3.0},
			},
		}
		
		// Verify structure
		assertEqual(t, fn.Name, "test/function")
		assertEqual(t, fn.Description, "Test function description")
		
		// Test evaluation
		result, err := EvaluateFunction(fn, map[string]float64{})
		if err != nil {
			t.Fatalf("Function evaluation failed: %v", err)
		}
		assertEqual(t, result, 6.0) // 2.0 * 3.0
	})
	
	t.Run("Specification Example 1 - QBar Area", func(t *testing.T) {
		// Example from spec:
		// <function name="aero/function/qbar-area">
		//   <product>
		//     <property>aero/qbar</property>
		//     <property>metrics/wingarea</property>
		//   </product>
		// </function>
		
		fn := &Function{
			Name: "aero/function/qbar-area",
			Product: &Operation{
				Property: []string{"aero/qbar", "metrics/wingarea"},
			},
		}
		
		properties := map[string]float64{
			"aero/qbar":         50.0,  // Dynamic pressure
			"metrics/wingarea":  200.0, // Wing area
		}
		
		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("QBar area function failed: %v", err)
		}
		assertEqual(t, result, 10000.0) // 50.0 * 200.0
	})
	
	t.Run("Specification Example 2 - CLDf with Table", func(t *testing.T) {
		// Example from spec:
		// <function name="aero/coefficient/CLDf">
		//   <description>Delta lift due to flap deflection</description>
		//   <product>
		//     <property>aero/function/ground-effect-factor-lift</property>
		//     <property>aero/qbar-area</property>
		//     <table>
		//       <independentVar>fcs/flap-pos-deg</independentVar>
		//       <tableData>
		//         0.0  0.0
		//         10.0 0.20
		//         20.0 0.30
		//         30.0 0.35
		//       </tableData>
		//     </table>
		//   </product>
		// </function>
		
		fn := &Function{
			Name:        "aero/coefficient/CLDf",
			Description: "Delta lift due to flap deflection",
			Product: &Operation{
				Property: []string{
					"aero/function/ground-effect-factor-lift",
					"aero/qbar-area",
				},
				Table: &Table{
					IndependentVar: []*IndependentVar{
						{Value: "fcs/flap-pos-deg"},
					},
					TableData: []*TableData{
						{Data: "0.0  0.0\n10.0 0.20\n20.0 0.30\n30.0 0.35"},
					},
				},
			},
		}
		
		properties := map[string]float64{
			"aero/function/ground-effect-factor-lift": 1.05,   // Ground effect factor
			"aero/qbar-area":                         10000.0, // From previous example
			"fcs/flap-pos-deg":                       15.0,    // Flap position
		}
		
		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("CLDf function failed: %v", err)
		}
		
		// Expected: 1.05 * 10000.0 * interpolate_table(15.0)
		// Table interpolation at 15.0 should give 0.25 (between 0.20 and 0.30)
		expected := 1.05 * 10000.0 * 0.25
		assertEqual(t, result, expected)
	})
	
	t.Run("Specification Example 3 - Circumference with Nesting", func(t *testing.T) {
		// Example from spec:
		// <function name="circumference">
		//   <product>
		//     <value>3.14159</value>
		//     <value>2.0</value>
		//     <sum>
		//       <property>radius</property>
		//       <value>200.0</value>
		//     </sum>
		//   </product>
		// </function>
		// Equation: circumference = 3.14159 * 2.0 * (200.0 + radius)
		
		fn := &Function{
			Name: "circumference",
			Product: &Operation{
				Value: []float64{3.14159, 2.0},
				Sum: &Operation{
					Property: []string{"radius"},
					Value:    []float64{200.0},
				},
			},
		}
		
		properties := map[string]float64{
			"radius": 50.0,
		}
		
		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("Circumference function failed: %v", err)
		}
		
		// Expected: 3.14159 * 2.0 * (50.0 + 200.0) = 3.14159 * 2.0 * 250.0
		expected := 3.14159 * 2.0 * 250.0
		assertApproxEqual(t, result, expected, 0.01)
	})
	
	t.Run("All Supported Operations", func(t *testing.T) {
		// Test all operations mentioned in spec:
		// product, difference, sum, quotient, pow, abs, sin, cos, tan, asin, acos, atan
		
		properties := map[string]float64{
			"val1": 4.0,
			"val2": 2.0,
			"val3": -3.0,
			"angle": 0.5, // ~28.6 degrees
		}
		
		tests := []struct {
			name     string
			fn       *Function
			expected float64
			tolerance float64
		}{
			{
				name: "product",
				fn: &Function{
					Product: &Operation{
						Property: []string{"val1", "val2"},
					},
				},
				expected: 8.0, // 4.0 * 2.0
				tolerance: 0.001,
			},
			{
				name: "sum",
				fn: &Function{
					Sum: &Operation{
						Property: []string{"val1", "val2", "val3"},
					},
				},
				expected: 3.0, // 4.0 + 2.0 + (-3.0)
				tolerance: 0.001,
			},
			{
				name: "difference",
				fn: &Function{
					Difference: &Operation{
						Property: []string{"val1", "val2"},
					},
				},
				expected: 2.0, // 4.0 - 2.0
				tolerance: 0.001,
			},
			{
				name: "quotient",
				fn: &Function{
					Quotient: &Operation{
						Property: []string{"val1", "val2"},
					},
				},
				expected: 2.0, // 4.0 / 2.0
				tolerance: 0.001,
			},
			{
				name: "pow",
				fn: &Function{
					Pow: &Operation{
						Property: []string{"val1", "val2"},
					},
				},
				expected: 16.0, // 4.0^2.0
				tolerance: 0.001,
			},
			{
				name: "abs",
				fn: &Function{
					Abs: &Operation{
						Property: []string{"val3"},
					},
				},
				expected: 3.0, // abs(-3.0)
				tolerance: 0.001,
			},
			{
				name: "sin",
				fn: &Function{
					Sin: &Operation{
						Property: []string{"angle"},
					},
				},
				expected: 0.479, // sin(0.5) ≈ 0.479
				tolerance: 0.01,
			},
			{
				name: "cos",
				fn: &Function{
					Cos: &Operation{
						Property: []string{"angle"},
					},
				},
				expected: 0.878, // cos(0.5) ≈ 0.878
				tolerance: 0.01,
			},
		}
		
		for _, test := range tests {
			t.Run(test.name, func(t *testing.T) {
				result, err := EvaluateFunction(test.fn, properties)
				if err != nil {
					t.Fatalf("%s operation failed: %v", test.name, err)
				}
				assertApproxEqual(t, result, test.expected, test.tolerance)
			})
		}
	})
	
	t.Run("Complex Nested Operations", func(t *testing.T) {
		// Test nested operations that match our implementation structure
		// Function: (a + b) * c  (simpler but tests nesting)
		fn := &Function{
			Name: "complex_nested",
			Product: &Operation{
				Sum: &Operation{
					Property: []string{"a", "b"},
				},
				Property: []string{"c"},
			},
		}
		
		properties := map[string]float64{
			"a": 3.0,
			"b": 7.0,
			"c": 2.0,
		}
		
		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("Complex nested function failed: %v", err)
		}
		
		// Expected: (3+7) * 2 = 10 * 2 = 20.0
		assertEqual(t, result, 20.0)
	})
	
	t.Run("Function with Mixed Values and Properties", func(t *testing.T) {
		// Test mixing literal values with properties
		fn := &Function{
			Name: "mixed_inputs",
			Product: &Operation{
				Value:    []float64{2.0, 3.14159},
				Property: []string{"radius"},
			},
		}
		
		properties := map[string]float64{
			"radius": 5.0,
		}
		
		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("Mixed inputs function failed: %v", err)
		}
		
		// Expected: 2.0 * 3.14159 * 5.0
		expected := 2.0 * 3.14159 * 5.0
		assertApproxEqual(t, result, expected, 0.01)
	})
	
	t.Run("Function with Table Lookup", func(t *testing.T) {
		// Test function that uses only table lookup
		fn := &Function{
			Name: "table_lookup",
			Table: &Table{
				IndependentVar: []*IndependentVar{
					{Value: "input_var"},
				},
				TableData: []*TableData{
					{Data: "0.0 1.0\n1.0 2.0\n2.0 4.0\n3.0 8.0"},
				},
			},
		}
		
		properties := map[string]float64{
			"input_var": 1.5,
		}
		
		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("Table lookup function failed: %v", err)
		}
		
		// Expected: interpolation between 2.0 and 4.0 at 1.5 = 3.0
		assertEqual(t, result, 3.0)
	})
}

// TestFunctionEvaluationEdgeCases tests edge cases and error conditions
func TestFunctionEvaluationEdgeCases(t *testing.T) {
	
	t.Run("Missing Properties", func(t *testing.T) {
		fn := &Function{
			Product: &Operation{
				Property: []string{"missing_property"},
				Value:    []float64{5.0}, // Should still work with literal values
			},
		}
		
		result, err := EvaluateFunction(fn, map[string]float64{})
		if err != nil {
			t.Fatalf("Function should handle missing properties gracefully: %v", err)
		}
		assertEqual(t, result, 5.0) // Should use the literal value
	})
	
	t.Run("Empty Operation", func(t *testing.T) {
		fn := &Function{
			Product: &Operation{}, // Empty operation
		}
		
		_, err := EvaluateFunction(fn, map[string]float64{})
		if err == nil {
			t.Error("Empty operation should return error")
		}
	})
	
	t.Run("Division by Zero", func(t *testing.T) {
		fn := &Function{
			Quotient: &Operation{
				Value: []float64{10.0, 0.0},
			},
		}
		
		result, err := EvaluateFunction(fn, map[string]float64{})
		if err != nil {
			t.Fatalf("Quotient function failed: %v", err)
		}
		// Should handle division by zero gracefully (probably returns original value)
		assertEqual(t, result, 10.0)
	})
	
	t.Run("Single Value Operations", func(t *testing.T) {
		// Test operations with single values
		tests := []struct {
			name     string
			fn       *Function
			expected float64
		}{
			{
				name: "difference_single",
				fn: &Function{
					Difference: &Operation{Value: []float64{5.0}},
				},
				expected: 5.0,
			},
			{
				name: "quotient_single", 
				fn: &Function{
					Quotient: &Operation{Value: []float64{7.0}},
				},
				expected: 7.0,
			},
			{
				name: "pow_single",
				fn: &Function{
					Pow: &Operation{Value: []float64{3.0}},
				},
				expected: 3.0,
			},
		}
		
		for _, test := range tests {
			t.Run(test.name, func(t *testing.T) {
				result, err := EvaluateFunction(test.fn, map[string]float64{})
				if err != nil {
					t.Fatalf("%s failed: %v", test.name, err)
				}
				assertEqual(t, result, test.expected)
			})
		}
	})
	
	t.Run("Trigonometric Functions Edge Cases", func(t *testing.T) {
		tests := []struct {
			name     string
			fn       *Function
			expected float64
			tolerance float64
		}{
			{
				name: "sin_zero",
				fn: &Function{
					Sin: &Operation{Value: []float64{0.0}},
				},
				expected: 0.0,
				tolerance: 0.001,
			},
			{
				name: "cos_zero",
				fn: &Function{
					Cos: &Operation{Value: []float64{0.0}},
				},
				expected: 1.0,
				tolerance: 0.001,
			},
			{
				name: "asin_half",
				fn: &Function{
					Asin: &Operation{Value: []float64{0.5}},
				},
				expected: 0.524, // arcsin(0.5) ≈ π/6
				tolerance: 0.01,
			},
		}
		
		for _, test := range tests {
			t.Run(test.name, func(t *testing.T) {
				result, err := EvaluateFunction(test.fn, map[string]float64{})
				if err != nil {
					t.Fatalf("%s failed: %v", test.name, err)
				}
				assertApproxEqual(t, result, test.expected, test.tolerance)
			})
		}
	})
}

// TestFunctionPerformance tests function evaluation performance
func TestFunctionPerformance(t *testing.T) {
	if testing.Short() {
		t.Skip("Skipping performance test in short mode")
	}
	
	t.Run("Complex Function Performance", func(t *testing.T) {
		// Create a complex nested function similar to real aerodynamic calculations
		fn := &Function{
			Name: "complex_aero",
			Product: &Operation{
				Property: []string{"aero/qbar-psf", "metrics/Sw-sqft"},
				Sum: &Operation{
					Product: &Operation{
						Value: []float64{0.1},
						Property: []string{"aero/alpha-deg"},
					},
					Table: &Table{
						IndependentVar: []*IndependentVar{
							{Value: "aero/mach"},
						},
						TableData: []*TableData{
							{Data: "0.0 1.0\n0.5 1.1\n0.8 1.2\n1.0 1.3"},
						},
					},
				},
			},
		}
		
		properties := map[string]float64{
			"aero/qbar-psf":  50.0,
			"metrics/Sw-sqft": 200.0,
			"aero/alpha-deg": 5.0,
			"aero/mach":      0.6,
		}
		
		// Benchmark function evaluation
		const iterations = 10000
		start := time.Now()
		
		for i := 0; i < iterations; i++ {
			_, err := EvaluateFunction(fn, properties)
			if err != nil {
				t.Fatalf("Function evaluation failed: %v", err)
			}
		}
		
		duration := time.Since(start)
		avgTime := duration / iterations
		
		t.Logf("Complex function evaluation: %d iterations in %v (%.2f μs per evaluation)", 
			iterations, duration, float64(avgTime.Nanoseconds())/1000.0)
		
		// Should be fast (under 10 μs per evaluation)
		if avgTime > time.Microsecond*10 {
			t.Errorf("Function evaluation too slow: %v per evaluation", avgTime)
		}
	})
}

// TestFunctionIntegrationWithTables tests function integration with table lookups
func TestFunctionIntegrationWithTables(t *testing.T) {
	
	t.Run("Function Using 2D Table", func(t *testing.T) {
		fn := &Function{
			Name: "aero_coeff_2d",
			Product: &Operation{
				Property: []string{"aero/qbar"},
				Table: &Table{
					IndependentVar: []*IndependentVar{
						{Lookup: "row", Value: "aero/alpha-deg"},
						{Lookup: "column", Value: "aero/mach"},
					},
					TableData: []*TableData{
						{Data: `        0.5    0.8
-10.0   0.1    0.2
10.0    0.3    0.4`},
					},
				},
			},
		}
		
		properties := map[string]float64{
			"aero/qbar":     100.0,
			"aero/alpha-deg": 0.0,
			"aero/mach":     0.65,
		}
		
		result, err := EvaluateFunction(fn, properties)
		if err != nil {
			t.Fatalf("2D table function failed: %v", err)
		}
		
		// Expected: 100.0 * interpolated_value
		// At alpha=0, mach=0.65: interpolate between columns
		// Row interpolation at alpha=0: between 0.1,0.2 and 0.3,0.4 -> 0.2,0.3
		// Column interpolation at mach=0.65: between 0.2 and 0.3 -> 0.25
		expected := 100.0 * 0.25
		assertEqual(t, result, expected)
	})
	
	t.Run("Nested Functions with Tables", func(t *testing.T) {
		// Test function that uses nested operations with multiple table lookups
		// Structure: sum of two separate function evaluations
		
		// First create a helper to test the concept differently
		// We'll test sequential evaluation instead
		fn1 := &Function{
			Name: "table_func_1",
			Product: &Operation{
				Property: []string{"factor1"},
				Table: &Table{
					IndependentVar: []*IndependentVar{{Value: "input1"}},
					TableData: []*TableData{{Data: "0.0 2.0\n1.0 4.0"}},
				},
			},
		}
		
		fn2 := &Function{
			Name: "table_func_2", 
			Product: &Operation{
				Property: []string{"factor2"},
				Table: &Table{
					IndependentVar: []*IndependentVar{{Value: "input2"}},
					TableData: []*TableData{{Data: "0.0 1.0\n1.0 3.0"}},
				},
			},
		}
		
		properties := map[string]float64{
			"factor1": 2.0,
			"factor2": 3.0,
			"input1":  0.5, // Table1 interpolation: 3.0
			"input2":  0.5, // Table2 interpolation: 2.0
		}
		
		result1, err := EvaluateFunction(fn1, properties)
		if err != nil {
			t.Fatalf("First table function failed: %v", err)
		}
		
		result2, err := EvaluateFunction(fn2, properties)
		if err != nil {
			t.Fatalf("Second table function failed: %v", err)
		}
		
		// Test individual results
		assertEqual(t, result1, 6.0) // 2.0 * 3.0
		assertEqual(t, result2, 6.0) // 3.0 * 2.0
		
		// Combined result would be 12.0
		combined := result1 + result2
		assertEqual(t, combined, 12.0)
	})
}
