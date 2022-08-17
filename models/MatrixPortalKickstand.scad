
OOZE_TOL=0.55;
battery_holder_thickness=2;
M3_SCREW_DIA=2.85+OOZE_TOL;
M3_SCREW_HEAD_DIA=5.52 + OOZE_TOL;
M3_WASHER_DIA=6.88 + OOZE_TOL;
M3_SCREW_HEAD_HEIGHT=2.88;
M3_HEXNUT_HEIGHT=2.47;
M3_HEXNUT_DIA=6 + OOZE_TOL;
HINGE_DIA=9;
height=42.94;
//height=33.18;
kickstand_thickness=3;
kickstand_width=100;
kickstand_centering_over_hinge=HINGE_DIA/2 - kickstand_thickness;// + battery_holder_thickness;
kickstand_angle=15;
HINGE_SEGMENT_WIDTH=4.75;
HINGE_SEGMENT_TOL=OOZE_TOL; // nozzle width 0.4 + extra ooze
tol=0.001; // used for rendering artifacts
BRACKET_WIDTH=12;
BRACKET_HEIGHT=123.5;
BRACKET_THICKNESS=2;

DEBUG=1; // set to 0 for printing.
FASTPRINT=0;
separation=9;

$fn=50;
IN_TO_MM=25.4;


module battry_holder_atom_tech() {
  // you get about >12mm to work with, the battery is 9mm thick
  
  height=42.94; // assumes height == 42.94...  todo, make this work for other heights.
  BATTERY_LENGTH=86;
  BATTERY_WIDTH=54.2;
  BATTERY_THICKNESS=9;
  DIST_to_next_BEAM=50;
  BEAM_WIDTH=10.2;
  /*
  DIST_from_BRACKET=25.8;
  ARM_WIDTH=1.5;
  ARM_HEIGHT=0.5 * IN_TO_MM;
  ARM_LENGTH=BATTERY_LENGTH + DIST_from_BRACKET;
  DIST_from_PLUG=22;
  BEAM_WIDTH=10.2;
  FLOOR_THICKNESS=0.4;
  DIST_to_USBPORT=2.35; // dist to the USB plug from side of battery
  */
  scale([-1,1,1])
  translate([BRACKET_WIDTH,0,-HINGE_DIA/2]) {
    difference() {
      union() {
        // top wall/grabber
        //translate([0,0,BATTERY_WIDTH+ARM_WIDTH])
        //scale([1,-1,1])
        //cube([ARM_LENGTH, ARM_HEIGHT, ARM_WIDTH]);

        // bottom wall/grabber
        //scale([1,-1,1])
        //cube([ARM_LENGTH, ARM_HEIGHT, ARM_WIDTH]);
        
        // floor
        //translate([0,0,ARM_WIDTH])
        //scale([1,-1,1])
        //cube([ARM_LENGTH, 0.4, DIST_to_USBPORT]);
        
        // battery
        color("orange")
        translate([DIST_to_next_BEAM + BEAM_WIDTH/2 - BATTERY_WIDTH/2, 0, BRACKET_THICKNESS])
        cube([BATTERY_WIDTH, BATTERY_LENGTH, BATTERY_THICKNESS]);
        
        // floor
        cube([DIST_to_next_BEAM + BEAM_WIDTH/2 + BATTERY_WIDTH/2, BATTERY_LENGTH, BRACKET_THICKNESS]);
        
      }

      union() {        
        //translate([DIST_from_BRACKET, 0, ARM_WIDTH])
        //scale([1,-1,1])
        //cube([BATTERY_LENGTH, 0.4, BATTERY_WIDTH]);
      }
    }
  }
}

module kick_flap( just_hinge ) {    
  if (just_hinge==false) {
  }

  rotate([-90,0,0])
  difference() {
    union() {
      cylinder( height, HINGE_DIA/2, HINGE_DIA/2 );
      
      if (just_hinge==false) {
        // flat part
        translate([0,kickstand_centering_over_hinge,0])
        cube([kickstand_width,kickstand_thickness,height]);
        
        // add a taper ramping down from the hinges to the flat part.
        difference() {
          // thick as the hinges
          translate([0,-HINGE_DIA/2,0])
          cube([BRACKET_WIDTH,HINGE_DIA,height]);
          
          // taper the thick
          translate([0,-HINGE_DIA/2,0])
          rotate([0,0, 25])
          scale([1,-1,1])
          cube([100,HINGE_DIA,height]);
        }
      }
      
      // mount
      if (just_hinge==true) {
        // flat part
        translate([0,HINGE_DIA/2,0])
        rotate([0,0,180])
        cube([BRACKET_WIDTH,BRACKET_THICKNESS,BRACKET_HEIGHT]);
        
        // add a taper ramping down from the hinges to the flat part.
        difference() {
          // thick as the hinges
          translate([0,HINGE_DIA/2,0])
          rotate([0,0,180])
          cube([BRACKET_WIDTH,HINGE_DIA,height]);
          
          // taper the thick
          translate([0,HINGE_DIA/2 - HINGE_DIA,0])
          rotate([0,0,180 - 25])
          cube([100,HINGE_DIA,height]);
        }
      }
    }
    
    // negative space
    union() {
      // kickstand top angle - negative space
      translate([HINGE_DIA/2, 0, 0])
      rotate([0, kickstand_angle/2, 0])
      translate([0, -kickstand_thickness*2, 0])
      translate([0,kickstand_centering_over_hinge,height+2.3])
      cube([kickstand_width*20,kickstand_thickness*6,height]);


      // kickstand bottom angle - negative space
      translate([HINGE_DIA/2 + 20, 0, 0])
      rotate([0, -kickstand_angle, 0])
      translate([0, -kickstand_thickness*2, 0])
      translate([0,kickstand_centering_over_hinge,-height])
      cube([kickstand_width*20,kickstand_thickness*6,height]);
    
      // screw head - negative space
      translate([0, 0, height - M3_SCREW_HEAD_HEIGHT])
      cylinder( M3_SCREW_HEAD_HEIGHT, M3_SCREW_HEAD_DIA/2, M3_SCREW_HEAD_DIA/2 );
      cylinder( height, M3_SCREW_DIA/2, M3_SCREW_DIA/2 );

      // hexnut - negative space
      translate([0, 0, 0])
      rotate([0,0,30])
      cylinder( M3_HEXNUT_HEIGHT, M3_HEXNUT_DIA/2, M3_HEXNUT_DIA/2, $fn = 6 ); // hex
      
      // hinge cylinder gaps - negative space
      for (a =[0:HINGE_SEGMENT_WIDTH*2:height]) {
        translate([0,0,a + (just_hinge ? HINGE_SEGMENT_WIDTH : 0)])
        cylinder(HINGE_SEGMENT_WIDTH+HINGE_SEGMENT_TOL, HINGE_DIA/2+tol+HINGE_SEGMENT_TOL, HINGE_DIA/2+tol+HINGE_SEGMENT_TOL);
      }

      // mounting holes for the bracket
      if (just_hinge==true) {
        translate([-8, 15, 5.7])
        rotate([90, 0, 0])
        cylinder( 30, M3_SCREW_DIA/2, M3_SCREW_DIA/2 );

        translate([-8, 15, 112 + 5.7])
        rotate([90, 0, 0])
        cylinder( 30, M3_SCREW_DIA/2, M3_SCREW_DIA/2 );
        
        translate([-8, HINGE_DIA/2 - BRACKET_THICKNESS, 5.7])
        rotate([90, 0, 0])
        cylinder( 30, M3_WASHER_DIA/2, M3_WASHER_DIA/2 );
      }
      
      // 
      //if (just_hinge==true) {
        
      //}
    }
  }
  
}


scale([-1,1,1])
difference() {
  union() {
    difference() {

      union() {
        //translate([0, 4, 0])
        //battery_holder();

        kick_flap( true );
        
        //battry_holder_atom_tech();

        //translate([0,HINGE_DIA/2,0])
        //rotate([0,0,180])
        //cube([kickstand_width,kickstand_thickness,height]);
      }

    }

    translate([separation,0.0]) {
      kick_flap(false);
    }
  }
  
  // debugging, reduce plastic so i can do quick tests
  union() {
    if (FASTPRINT) {
      translate([-20,height, -10])
      cube([20,130,20]);

      translate([separation + HINGE_DIA/2 + 7,0, -10])
      cube([300,height,20]);
    }
  }
}



