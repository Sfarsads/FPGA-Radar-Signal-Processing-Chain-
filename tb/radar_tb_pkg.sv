// radar_tb_pkg.sv: testbench-only shared state (a package variable, not a hierarchical reference).
// radar_sva.sv increments sva_fail_count on every assertion failure; radar_tb.sv reads it once at the
// end of the run. Using a package variable instead of a `dut.u_sva.fails`-style path avoids simulator
// differences in how `bind`-created instances are named up during optimization/elaboration.
package radar_tb_pkg;
  int unsigned sva_fail_count = 0;
endpackage
