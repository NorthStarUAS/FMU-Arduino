#pragma once

// a class to implement a simple low pass filter

class rcLowPassFilter {
    
private:
    
    // Time factor (tf): length of time (sec) to low pass filter the
    // input over.  A time value of zero will result in the filter
    // output being equal to the raw input at each time step.
    double _time_factor = 1.0;

    // the current filter value
    double filter_value = 0.0;

    // have we been inited
    bool inited = false;
    
public:
    
    rcLowPassFilter();
    rcLowPassFilter( double time_factor );
    rcLowPassFilter( double time_factor, double init );
    ~rcLowPassFilter();
    
    inline void set_time_factor( double time_factor ) {
	_time_factor = time_factor;
    }
    
    inline void init( double value ) {
	filter_value = value;
	inited = true;
    }
    
    double update( double value, double dt );
    
    inline double get_value() {
	return filter_value;
    }
};
