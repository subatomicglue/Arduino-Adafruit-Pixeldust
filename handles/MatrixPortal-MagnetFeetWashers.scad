
// print some washers to tighten the loose handle fit
// needed because when using the magnetic feet, they're just a bit too long and the handle is loose. 
height=1.2;
outer=10;
inner=6;
$fn=180;
difference() {
  cylinder( height, outer/2, outer/2 );
  cylinder( height, inner/2, inner/2 );
}