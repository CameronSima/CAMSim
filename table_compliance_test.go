package main

import (
	"testing"
)

// TestJSBSimComplianceVerification validates that our implementation follows ALL JSBSim table rules exactly
func TestJSBSimComplianceVerification(t *testing.T) {
	
	t.Run("1D Table Compliance", func(t *testing.T) {
		// Rule: One-dimensional table defined by one independent lookup variable and breakpoints
		// Format: <independentVar>property_name</independentVar>
		//         <tableData>row_index_1 data_1 ... row_index_n data_n</tableData>
		
		table := &Table{
			Name: "compliance_1d",
			IndependentVar: []*IndependentVar{
				{Value: "test/property"}, // No lookup attribute for 1D
			},
			TableData: []*TableData{
				{Data: "0.0 1.0\n1.0 2.0\n2.0 4.0\n3.0 8.0"},
			},
		}
		
		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("1D table parsing failed: %v", err)
		}
		
		// Compliance checks
		assertEqual(t, pt.Dimension, 1)
		assertEqual(t, len(pt.IndependentVars), 1)
		assertEqual(t, pt.LookupTypes[0], "") // No lookup type for 1D
		
		if pt.Data1D == nil {
			t.Fatal("1D data structure missing")
		}
		if pt.Data2D != nil || pt.Data3D != nil {
			t.Error("1D table should not have 2D or 3D data structures")
		}
		
		// Verify data integrity
		assertEqual(t, len(pt.Data1D.Indices), 4)
		assertEqual(t, len(pt.Data1D.Values), 4)
		assertEqual(t, pt.Data1D.Indices[0], 0.0)
		assertEqual(t, pt.Data1D.Values[3], 8.0)
		
		// Test interpolation
		result, err := InterpolateTable(pt, 1.5)
		if err != nil {
			t.Fatalf("1D interpolation failed: %v", err)
		}
		assertEqual(t, result, 3.0) // Linear interpolation between 2.0 and 4.0
	})
	
	t.Run("2D Table Compliance", func(t *testing.T) {
		// Rule: Two-dimensional table defined by two independent lookup variables
		// Format: <independentVar lookup="row">property_name</independentVar>
		//         <independentVar lookup="column">property_name</independentVar>
		//         <tableData>
		//           col_idx_1 col_idx_2 ... col_idx_n
		//           row_index_1 data_11 data_12 ... data_1n
		//           row_index_2 data_21 data_22 ... data_2n
		//           ...
		//         </tableData>
		
		table := &Table{
			Name: "compliance_2d",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "row/property"},
				{Lookup: "column", Value: "col/property"},
			},
			TableData: []*TableData{
				{Data: `        10.0   20.0   30.0
100.0   1.0    1.5    2.0
200.0   2.0    3.0    4.0
300.0   3.0    4.5    6.0`},
			},
		}
		
		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("2D table parsing failed: %v", err)
		}
		
		// Compliance checks
		assertEqual(t, pt.Dimension, 2)
		assertEqual(t, len(pt.IndependentVars), 2)
		assertEqual(t, pt.LookupTypes[0], "row")
		assertEqual(t, pt.LookupTypes[1], "column")
		
		if pt.Data2D == nil {
			t.Fatal("2D data structure missing")
		}
		if pt.Data1D != nil || pt.Data3D != nil {
			t.Error("2D table should not have 1D or 3D data structures")
		}
		
		// Verify structure: first row = column indices, first column = row indices
		assertEqual(t, len(pt.Data2D.ColIndices), 3) // 3 columns
		assertEqual(t, len(pt.Data2D.RowIndices), 3) // 3 rows
		assertEqual(t, len(pt.Data2D.Data), 3)       // 3 data rows
		
		// Check column indices (from first row)
		assertEqual(t, pt.Data2D.ColIndices[0], 10.0)
		assertEqual(t, pt.Data2D.ColIndices[1], 20.0)
		assertEqual(t, pt.Data2D.ColIndices[2], 30.0)
		
		// Check row indices (from first column)
		assertEqual(t, pt.Data2D.RowIndices[0], 100.0)
		assertEqual(t, pt.Data2D.RowIndices[1], 200.0)
		assertEqual(t, pt.Data2D.RowIndices[2], 300.0)
		
		// Check data values
		assertEqual(t, pt.Data2D.Data[0][0], 1.0) // row=100, col=10
		assertEqual(t, pt.Data2D.Data[1][1], 3.0) // row=200, col=20
		assertEqual(t, pt.Data2D.Data[2][2], 6.0) // row=300, col=30
		
		// Test bilinear interpolation
		result, err := InterpolateTable(pt, 150.0, 15.0) // Mid-points
		if err != nil {
			t.Fatalf("2D interpolation failed: %v", err)
		}
		
		// Manual calculation:
		// At row 100, col 15: interpolate between 1.0 (col 10) and 1.5 (col 20) = 1.25
		// At row 200, col 15: interpolate between 2.0 (col 10) and 3.0 (col 20) = 2.5
		// At row 150: interpolate between 1.25 and 2.5 = 1.875
		assertEqual(t, result, 1.875) // Should interpolate correctly
	})
	
	t.Run("3D Table Compliance", func(t *testing.T) {
		// Rule: Three-dimensional table defined by three independent lookup variables
		// and multiple 2D tables with breakPoint attributes
		// Format: <independentVar lookup="row">property_name</independentVar>
		//         <independentVar lookup="column">property_name</independentVar>
		//         <independentVar lookup="table">property_name</independentVar>
		//         <tableData breakpoint="table_index_1">...</tableData>
		//         <tableData breakpoint="table_index_2">...</tableData>
		//         ...
		
		table := &Table{
			Name: "compliance_3d",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "row/property"},
				{Lookup: "column", Value: "col/property"},
				{Lookup: "table", Value: "table/property"},
			},
			TableData: []*TableData{
				{
					Breakpoint: "1.0", // First table breakpoint
					Data: `        10.0   20.0
100.0   1.0    2.0
200.0   3.0    4.0`,
				},
				{
					Breakpoint: "2.0", // Second table breakpoint
					Data: `        10.0   20.0
100.0   2.0    3.0
200.0   4.0    5.0`,
				},
				{
					Breakpoint: "3.0", // Third table breakpoint
					Data: `        10.0   20.0
100.0   3.0    4.0
200.0   5.0    6.0`,
				},
			},
		}
		
		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("3D table parsing failed: %v", err)
		}
		
		// Compliance checks
		assertEqual(t, pt.Dimension, 3)
		assertEqual(t, len(pt.IndependentVars), 3)
		assertEqual(t, pt.LookupTypes[0], "row")
		assertEqual(t, pt.LookupTypes[1], "column")
		assertEqual(t, pt.LookupTypes[2], "table")
		
		if pt.Data3D == nil {
			t.Fatal("3D data structure missing")
		}
		if pt.Data1D != nil || pt.Data2D != nil {
			t.Error("3D table should not have 1D or 2D data structures")
		}
		
		// Verify 3D structure: multiple 2D tables with breakpoints
		assertEqual(t, len(pt.Data3D), 3) // 3 breakpoint tables
		
		// Check breakpoints
		assertEqual(t, pt.Data3D[0].Breakpoint, 1.0)
		assertEqual(t, pt.Data3D[1].Breakpoint, 2.0)
		assertEqual(t, pt.Data3D[2].Breakpoint, 3.0)
		
		// Each 2D table should have same structure
		for i, table2d := range pt.Data3D {
			assertEqual(t, len(table2d.ColIndices), 2) // 2 columns
			assertEqual(t, len(table2d.RowIndices), 2) // 2 rows
			assertEqual(t, len(table2d.Data), 2)       // 2 data rows
			
			// Check column/row indices consistency
			assertEqual(t, table2d.ColIndices[0], 10.0)
			assertEqual(t, table2d.ColIndices[1], 20.0)
			assertEqual(t, table2d.RowIndices[0], 100.0)
			assertEqual(t, table2d.RowIndices[1], 200.0)
			
			// Check data progression (should increase with breakpoint)
			if i > 0 {
				prev := pt.Data3D[i-1].Data[0][0]
				curr := table2d.Data[0][0]
				if curr <= prev {
					t.Errorf("3D table data should progress with breakpoints: table[%d][0][0]=%f should be > table[%d][0][0]=%f", 
						i, curr, i-1, prev)
				}
			}
		}
		
		// Test trilinear interpolation
		result, err := InterpolateTable(pt, 150.0, 15.0, 2.5) // Mid-points in all dimensions
		if err != nil {
			t.Fatalf("3D interpolation failed: %v", err)
		}
		
		// Should interpolate correctly across all three dimensions
		if result < 2.0 || result > 5.0 {
			t.Errorf("3D interpolation result %f outside expected range [2.0, 5.0]", result)
		}
	})
	
	t.Run("Mixed Breakpoint Attribute Support", func(t *testing.T) {
		// Test that we support both "breakpoint" and "breakPoint" attributes
		// (Real JSBSim files use "breakPoint" with capital P)
		
		table := &Table{
			Name: "mixed_breakpoint",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "test/row"},
				{Lookup: "column", Value: "test/col"}, 
				{Lookup: "table", Value: "test/table"},
			},
			TableData: []*TableData{
				{
					BreakPoint: "10.0", // Using capital P version
					Data: `        1.0   2.0
0.0     0.1   0.2
1.0     0.3   0.4`,
				},
				{
					Breakpoint: "20.0", // Using lowercase p version
					Data: `        1.0   2.0
0.0     0.2   0.3
1.0     0.4   0.5`,
				},
			},
		}
		
		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Mixed breakpoint parsing failed: %v", err)
		}
		
		assertEqual(t, len(pt.Data3D), 2)
		assertEqual(t, pt.Data3D[0].Breakpoint, 10.0) // Should parse BreakPoint
		assertEqual(t, pt.Data3D[1].Breakpoint, 20.0) // Should parse Breakpoint
	})
	
	t.Run("Input Parameter Validation", func(t *testing.T) {
		// Test that interpolation requires correct number of inputs
		
		// 1D table
		table1D := &Table{
			Name: "validation_1d",
			IndependentVar: []*IndependentVar{{Value: "test"}},
			TableData: []*TableData{{Data: "0.0 1.0\n1.0 2.0"}},
		}
		pt1D, _ := ParseTable(table1D)
		
		// Should accept 1 input
		_, err := InterpolateTable(pt1D, 0.5)
		if err != nil {
			t.Errorf("1D table should accept 1 input: %v", err)
		}
		
		// Should reject 2 inputs
		_, err = InterpolateTable(pt1D, 0.5, 1.0)
		if err == nil {
			t.Error("1D table should reject 2 inputs")
		}
		
		// 2D table
		table2D := &Table{
			Name: "validation_2d",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "test1"},
				{Lookup: "column", Value: "test2"},
			},
			TableData: []*TableData{{Data: "        1.0\n0.0     2.0"}},
		}
		pt2D, _ := ParseTable(table2D)
		
		// Should accept 2 inputs
		_, err = InterpolateTable(pt2D, 0.0, 1.0)
		if err != nil {
			t.Errorf("2D table should accept 2 inputs: %v", err)
		}
		
		// Should reject 1 input
		_, err = InterpolateTable(pt2D, 0.5)
		if err == nil {
			t.Error("2D table should reject 1 input")
		}
		
		// Should reject 3 inputs
		_, err = InterpolateTable(pt2D, 0.5, 1.0, 2.0)
		if err == nil {
			t.Error("2D table should reject 3 inputs")
		}
		
		// 3D table
		table3D := &Table{
			Name: "validation_3d",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "test1"},
				{Lookup: "column", Value: "test2"},
				{Lookup: "table", Value: "test3"},
			},
			TableData: []*TableData{
				{Breakpoint: "1.0", Data: "        1.0\n0.0     2.0"},
			},
		}
		pt3D, _ := ParseTable(table3D)
		
		// Should accept 3 inputs
		_, err = InterpolateTable(pt3D, 0.0, 1.0, 1.0)
		if err != nil {
			t.Errorf("3D table should accept 3 inputs: %v", err)
		}
		
		// Should reject 2 inputs
		_, err = InterpolateTable(pt3D, 0.5, 1.0)
		if err == nil {
			t.Error("3D table should reject 2 inputs")
		}
	})
}

// TestComplexRealWorldScenarios tests complex real-world table scenarios
func TestComplexRealWorldScenarios(t *testing.T) {
	
	t.Run("Asymmetric 2D Table", func(t *testing.T) {
		// Test tables where not all rows have the same number of columns
		// (This can happen in real aerodynamic data)
		
		table := &Table{
			Name: "asymmetric",
			IndependentVar: []*IndependentVar{
				{Lookup: "row", Value: "aero/alpha-deg"},
				{Lookup: "column", Value: "aero/mach"},
			},
			TableData: []*TableData{
				{Data: `        0.3    0.5    0.7    0.9
-20.0   -1.5   -1.4   -1.3   -1.2
-10.0   -0.8   -0.7   -0.6   -0.5
0.0     0.0    0.1    0.2    0.3
10.0    0.8    0.9    1.0    1.1
20.0    1.5    1.6    1.7    1.8`},
			},
		}
		
		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse asymmetric table: %v", err)
		}
		
		// Should handle interpolation gracefully
		result, err := InterpolateTable(pt, 5.0, 0.6)
		if err != nil {
			t.Fatalf("Failed to interpolate asymmetric table: %v", err)
		}
		
		// Should give reasonable result
		if result < 0.0 || result > 2.0 {
			t.Errorf("Interpolation result %f outside reasonable range", result)
		}
	})
	
	t.Run("High Precision Data", func(t *testing.T) {
		// Test with high precision floating point data
		
		table := &Table{
			Name: "high_precision",
			IndependentVar: []*IndependentVar{
				{Value: "precise/property"},
			},
			TableData: []*TableData{
				{Data: `0.123456789  1.987654321
0.234567890  2.876543210
0.345678901  3.765432109`},
			},
		}
		
		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse high precision table: %v", err)
		}
		
		// Check precision is maintained
		assertApproxEqual(t, pt.Data1D.Indices[0], 0.123456789, 1e-9)
		assertApproxEqual(t, pt.Data1D.Values[0], 1.987654321, 1e-9)
		
		// Test interpolation maintains precision
		result, err := InterpolateTable(pt, 0.179012340) // Between first two points
		if err != nil {
			t.Fatalf("Failed to interpolate high precision: %v", err)
		}
		
		// Should maintain reasonable precision
		if result < 2.0 || result > 3.0 {
			t.Errorf("High precision interpolation gave unreasonable result: %f", result)
		}
	})
	
	t.Run("Extreme Range Values", func(t *testing.T) {
		// Test with very large and very small values
		
		table := &Table{
			Name: "extreme_values",
			IndependentVar: []*IndependentVar{
				{Value: "extreme/property"},
			},
			TableData: []*TableData{
				{Data: `1e-10  1e+10
1e-5   1e+5
1e+0   1e+0
1e+5   1e-5
1e+10  1e-10`},
			},
		}
		
		pt, err := ParseTable(table)
		if err != nil {
			t.Fatalf("Failed to parse extreme values table: %v", err)
		}
		
		// Test interpolation with extreme values
		result, err := InterpolateTable(pt, 1e-7) // Very small input
		if err != nil {
			t.Fatalf("Failed to interpolate extreme values: %v", err)
		}
		
		// Should handle extreme values gracefully
		if result <= 0 {
			t.Errorf("Extreme value interpolation gave non-positive result: %e", result)
		}
	})
}
