use <smooth_prim.scad>;
use <BOSL/transforms.scad>

module top_hat() {
        
    height = 20;
    width = 90;
    deep = 35;

    difference() {
        yrot(90) translate([-20, 0,0]) SmoothXYCube([height, deep, width], 2);
        
        translate([-1, -1 ,-1]) cube([width+2, deep+2, 15+1]);
        
        translate([75, deep/2, 0]) cylinder(30, 5, 5);
        translate([75, deep/2, 14]) cube([20, 20, 5+1], center=true);
        
        translate([10, 20 , 16]) cube([50, 7, 7]);
        translate([8, 18 , 18]) cube([54, 12, 3]);
    }
    
}
top_hat();