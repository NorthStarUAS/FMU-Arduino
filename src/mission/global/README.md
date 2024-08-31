# Notes on mission code

## Waypoints

There are two types of waypoints in the mission manager library.

* Relative wpts that store a direction and distance relative to the "home"
  location.  When the home position moves, the route (pattern) is repositioned
  relative to the new home location.  This is a special use cases that arose out
  of the need to do ship-based surveys ... where if you spent 30 minutes
  programming a static route and uploading it, the ship may have drifted a mile
  by then.  Thes are also nice to fly test patterns that just work whereever you
  power on and fly.
* Absolute wpts (more common.)  This is the result of drawing a route on
  the map, or creating a survey area.

It would make sense to create a base class + derived classes to centralize the
route management code.  So create all the wpt intances as pointers to
dynamically allocated instances so the runtime binding works.  However, memory
management is a big concern and exhausting onboard memory could crash the plane
so I am avoiding the obvious/natural C++ way to set this up, and going for a
more clunky approach.  Lots of pointers and dynamic memory allocation in a real
time memory limited flight critical system is like walking way up the very edge
of a cliff ... it doesn't take much then for things to go wrong!

This is why the route management code may end up looking a bit weird: optimizing
mimimal dynamic memory allocation, minimal pointers, minimal chance of nullptr
references, etc.  I do know there are fancier C++ ways to approach this, but I
have a viceral reaction against the crazy convoluted syntax that involves,
sorry, I'm an old school C programmer that learned how to do basic classes
somewhere along the way I and like to keep my code simple!  I'd rather be doing
this in python anyway. (smiley face)

## Waypoint Attributes

Shouldn't each waypoint store more than a 2d location?  Wouldn't I want to
include reference altitude, speed, maybe a directive to trigger a camera
shutter, or fire off other actions?  So ... the answer to that is ... no!

Everyone's use cases differ, I get that.  For my own use cases I found that it
made the most sense to define circle and route actions in 2d only.  Then allow
the operator to independently set speed and altitude.  For surveys, the altitude
would get cooked into the survey and wouldn't need to change at each waypoint.

How do I do autoland if I can't bake altitudes into waypoints ... ugh, I always
thought that was a terrible way to do autoland, sorry.  I have a more dynamic
procedural way to compute landing approaches that I think is far nicer.

I have had to sit for 15 minutes to upload a long mission to arducopter, only to
have it fail mysteriously after 10 minutes.  And again on retry, and again on
retry, and finally succeed after 45 minutes of pain.  I don't like that, I won't
design this system that way!
