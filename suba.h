#include <string>

#ifndef SUBA_H
#define SUBA_H

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subatomic's Math and Filters
////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef M_PI
#define M_PI		3.14159265358979323846f
#endif

// subatomic namespace
namespace suba
{
  ///////////////////////////////////////////////////////////////////////
  // scalar math

  //  with places == 1,  10.489478 becomes 10.4
  double truncate( double value, int places ) {
    double p = pow( 10.0, places );
    return floor( value * p ) / p;
  }
  
  /** min returns the minimum of 2 values */
  template <class T>
  inline T Min( const T x, const T y )
  {
     return ( x > y ) ? y : x;
  }
  /** max returns the maximum of 2 values */
  template <class T>
  inline T Max( const T x, const T y )
  {
     return ( x > y ) ? x : y;
  }

  /// clamp between two values.
  template <class T>
  inline T clamp( const T x, const T low, const T high )
  {
      // no measurable difference between these methods on intel p4 2.26ghz x86 family 15 model 2...
     //   if ( x < low ) return low;
     //   else if (high < x) return high;
     //   else return x;
     return suba::Min( high, suba::Max( x, low ) ); // leaving this one in since min/max are intrinsics on some platforms, and because conditionals are really expensive on some platforms...
  }

  inline float unitRandom()
  {
     // return between 0 and 1
     return float(rand())/float(RAND_MAX);
  }

  /** return a random number between x1 and x2
   * RETURNS: random number between x1 and x2
   */
  inline float rangeRandom( float x1, float x2 )
  {
     float r = suba::unitRandom();
     float size = x2 - x1;
     return float( r * size + x1 );
  }

  template <typename T>
  inline T rangeRandom( T x1, T x2 )
  {
     return (T)rangeRandom( float( x1 ), float( x2 ) );
  }

  // seems one of the other headers already has round() #defined...
  inline double roundToNearestInt( double p ) { return floor( p + 0.5 ); }

  /// halving (or some other fraction) smoothing routine (iterative result).  result is an exponential curve...
  /// filters an input value towards some end value, cutting it in [factor] each time. (usually an abrupt fade in, then ramp up slow)
  /// input range [0..1], outputs range [0..1]
  inline float smoothGoal( float currentvalue, float goalvalue, float factor )
  {
      const float den = 1e-15f; // avoid denormal numbers, on intel these kinds of numbers suck cpu %
      const float dist_to_end = goalvalue - currentvalue;
      const float stride_this_frame = dist_to_end * factor + den;
      return currentvalue + stride_this_frame;
  }

  // linear interpolate
  double lerp( double a, double b, double t ) {
    return ((1.0 - t) * a) + (t * b);
  }

  ///////////////////////////////////////////////////////////////////////
  // 3D vector
  struct Vec3 {
    Vec3() { d[0]=0.0; d[1]=0.0; d[2]=0.0; }
    Vec3( double x, double y, double z ) { d[0]=x; d[1]=y; d[2]=z; }
    Vec3( const Vec3& v ) { d[0]=v[0]; d[1]=v[1]; d[2]=v[2]; }
    double& operator[]( size_t i ) { return d[i]; }
    const double& operator[]( size_t i ) const { return d[i]; }
    double d[3];
  };

  // vector normalize
  Vec3 normalize( const Vec3& v ) {
    double lr = 1.0 / sqrt( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
    return Vec3( v[0] * lr, v[1] * lr, v[2] * lr );
  }

  // vector dot product
  double dot( const Vec3& v1, const Vec3& v2 ) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
  }

  // acos of the dotproduct (angle between two normalized vectors, radians)
  double dot_angle( const Vec3& v1, const Vec3& v2 ) {
    return acos( dot( normalize( v1 ), normalize( v2 ) ) );
  }

  // vector cross product
  Vec3 cross( const Vec3& a, const Vec3& b ) {
    return Vec3( a[1] * b[2] - a[2] * b[1],
                 a[2] * b[0] - a[0] * b[2],
                 a[0] * b[1] - a[1] * b[0] );
  }

  // vector subtract
  Vec3 sub( const Vec3& v1, const Vec3& v2 ) {
    return Vec3( v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2] );
  }

  // vector add
  Vec3 add( const Vec3& v1, const Vec3& v2 ) {
    return Vec3( v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2] );
  }

  // vector multiply by scalar
  Vec3 mul( const Vec3& v, double s ) {
    return Vec3( v[0] * s, v[1] * s, v[2] * s );
  }
  
  // vector linear interpolate
  suba::Vec3 lerp( const suba::Vec3& a, const suba::Vec3& b, double t ) {
    return suba::Vec3( lerp( a[0], b[0], t ),
                 lerp( a[1], b[1], t ),
                 lerp( a[2], b[2], t ) );
  }

  // vector spherical interpolation from https://stackoverflow.com/questions/67919193/how-does-unity-implements-vector3-slerp-exactly
  inline Vec3 slerp( const Vec3& start, const Vec3& end, float percent )
  {
     // Dot product - the cosine of the angle between 2 (normalized) vectors.
     float dp = dot( start, end );

     // Clamp it to be in the range of Acos()
     // This may be unnecessary, but floating point
     // precision can be a fickle mistress.
     suba::clamp( dp, -1.0f, 1.0f );
  
     // Acos(dot) returns the angle between start and end,
     // And multiplying that by percent returns the angle between
     // start and the final result.
     float theta = acos(dp) * percent;
     Vec3 relative_vec = sub( end, mul( start, dp ) );
     relative_vec = normalize( relative_vec );
  
     // Orthonormal basis
     // The final result.
     return add( mul( start, cos(theta)), mul( relative_vec, sin(theta) ) );
  }

  inline Vec3 smoothGoal( const Vec3& currentvalue, const Vec3& goalvalue, double factor ) {
    return Vec3( smoothGoal( currentvalue[0], goalvalue[0], factor ),
                 smoothGoal( currentvalue[1], goalvalue[1], factor ),
                 smoothGoal( currentvalue[2], goalvalue[2], factor ) );
  }



  ////////////////////////////////////////////////////////////////////////////////
  // FILTERS

  /// filter created by ove karlson (found on musicdsp.org in the mid 2000's)
  /// can be driven with a VCF...
  class OveKarlsonResFilter
  {
  public:
     OveKarlsonResFilter()
     {
         b_fp = 0.0f;
         pole1 = 0.0f;
         pole2 = 0.0f;
         pole3 = 0.0f;
         pole4 = 0.0f;
         gain = 0.5f;
         resonance = 0.0f;
         cutoff = 1.0f;
         fixedGain = 0.0f;
         mSampRate = 44100.0f;
         mAdjust[0] = 0.0f;
         mAdjust[1] = 0.0f;
         mAdjust[2] = 0.0f;
         mAdjust[3] = 0.0f;
  
         setCutoff( cutoff );
         setLowPass();
     }
     OveKarlsonResFilter( const OveKarlsonResFilter& proto )
     {
         b_fp = proto.b_fp;
         pole1 = proto.pole1;
         pole2 = proto.pole2;
         pole3 = proto.pole3;
         pole4 = proto.pole4;
         gain = proto.gain;
         resonance = proto.resonance;
         cutoff = proto.cutoff;
         fixedGain = proto.fixedGain;
         mSampRate = proto.mSampRate;
         mAdjust[0] = proto.mAdjust[0];
         mAdjust[1] = proto.mAdjust[1];
         mAdjust[2] = proto.mAdjust[2];
         mAdjust[3] = proto.mAdjust[3];
     }
  
     /// make this filter a high pass filter.
     inline void setHighPass()
     {
        mAdjust[0] = -1.0f; // todo: find out what's fastest, these multiplies, or 3 separate code paths in filter().
        mAdjust[1] = 1.0f;
        mAdjust[2] = 1.0f;
        mAdjust[3] = 1.0f;
     }
  
     /// make this filter a band pass filter.
     inline void setBandPass()
     {
        mAdjust[0] = -1.0f;
        mAdjust[1] = -1.0f;
        mAdjust[2] = -1.0f;
        mAdjust[3] = -1.0f;
     }
  
     /// make this filter a low pass filter.
     inline void setLowPass()
     {
        mAdjust[0] = 1.0f;
        mAdjust[1] = 1.0f;
        mAdjust[2] = 1.0f;
        mAdjust[3] = 1.0f;
     }
  
     inline void setSampRate( float samp )
     {
        mSampRate = samp;
     }
  
     
     inline void setGain( float data )
     {
  	   gain = suba::clamp( data, 0.0f, 1.0f );
     }
  
     inline void setCutoff(float data)
     {
  	   cutoff = suba::clamp( data, 0.001f, 0.999f );
  	}
     inline float getCutoff() const { return cutoff; }
     inline void setCutoffNoCheck(float data) { cutoff = data; }
  
     void setRes(float data)
     {
  	   data *= 0.125f; // div by 8, make it less sensitive to changes.
  	   resonance = suba::clamp( data, 0.001f, 0.999f );
  	}
  
     /// http://www.musicdsp.org/showone.php?id=141
     /// http://www.musicdsp.org/showArchiveComment.php?ArchiveID=141
     /// http://www.google.com/search?hl=en&ie=UTF-8&q=Ove+Karlsen&btnG=Google+Search
     /// Karlsen
     /// Type : 24-dB (4-pole) lowpassReferences : Posted by Best Regards,Ove Karlsen
     /// Notes :
     /// There's really not much voodoo going on in the filter itself, it's a simple as possible:
     /// 
     /// pole1 = (in * frequency) + (pole1 * (1 - frequency));
     /// 
     /// Most of you can probably understand that math, it's very similar to how an analog condenser works.
     /// Although, I did have to do some JuJu to add resonance to it.
     /// While studing the other filters, I found that the feedback phase is very important to how the overall
     /// resonance level will be, and so I made a dynamic feedback path, and constant Q approximation by manipulation
     /// of the feedback phase.
     /// A bonus with this filter, is that you can "overdrive" it... Try high input levels..
     inline float filter( float data )
     {
  	   /*
  	   automatic slide adjustment. prevents strange filter effects
  	   */
        // Karlsen 24dB Filter by Ove Karlsen / Synergy-7 in the year 2003.
        // b_f = frequency 0..1
        // b_q = resonance 0..50
        // b_in = input
        // to do bandpass, subtract poles from eachother, highpass subtract with input.
        float b_in = data; // before the while statement.
        const float b_f = cutoff;
        const float one_minus_b_f = 1.0f - b_f;
        const float b_q = resonance * 30.6227f;
        const float b_inSH = b_in;
        for (int b_oversample = 0; b_oversample < 2; ++b_oversample) 
        {
           //2x oversampling (@44.1khz)
           //float prevfp = b_fp;
           //if (prevfp > 1.0f) {prevfp = 1.0f;}                      // Q-limiter
           float prevfp = suba::Min( 1.0f, b_fp );      // Q-limiter
  
           b_fp = (b_fp * 0.418f) + ((b_q * pole4) * 0.582f);         // dynamic feedback
           float intfp = (b_fp * 0.36f) + (prevfp * 0.64f);  // feedback phase
           b_in =    b_inSH - intfp;                                  // inverted feedback
  
           pole1 = (b_in    * b_f) + mAdjust[0] * (pole1 * (one_minus_b_f));            // pole 1
  
  #ifndef SUBA_FILTER_DISABLE_CLAMPING
           // useful for audio signals from -1 to 1, enabled by default:
           //if (pole1 > 1.0f) {pole1 = 1.0f;} else if (pole1 < -1.0f) {pole1 = -1.0f;}  // pole 1 clipping (clamp value between [-1..1])
           pole1 = suba::clamp( pole1, -1.0f, 1.0f );                               // pole 1 clipping (clamp value between [-1..1])
  #endif
  
           pole2 = (pole1   * b_f) + mAdjust[1] * (pole2 * (one_minus_b_f));            // pole 2
           pole3 = (pole2   * b_f) + mAdjust[2] * (pole3 * (one_minus_b_f));            // pole 3
           pole4 = (pole3   * b_f) + mAdjust[3] * (pole4 * (one_minus_b_f));            // pole 4
        }
  	  return pole4;
     }
  
  private:
     float b_fp;
  	float pole1;
  	float pole2;
  	float pole3;
  	float pole4;
     float gain;
  	float resonance;
  	float cutoff;
     float fixedGain;
  
     float mSampRate;
  
     float mAdjust[4]; // coefs we use to adjust the filter from lowpass, highpass, bandpass.  simple [1] values + or -
  };
  
  class SimpleHighPass
  {
  public:
     SimpleHighPass() : mCutoff( 1.0f ), mSampRate( 44100.0f ), lastin( 0 ), lastout( 0 )
     {
         this->setCutoff( mCutoff, 44100.0f );
     }
  
     void setSampRate( float samp )
     {
        mSampRate = samp;
     }
  
     void setCutoff( float cutoff, float sampRate )
     {
        mCutoff = cutoff / mSampRate;
        if (mCutoff < 0.0f) mCutoff = 0.0f;
        if (mCutoff > 1.0f) mCutoff = 1.0f;
  
        a = 1.0f / ( 1.0f + 10.8827961853f * mCutoff );
     }
  
     inline float filter( float data )
     {
        lastout = a*lastout + data - lastin;
        lastin = data;
        return lastout;
     }
     
     float mSampRate;
     float mCutoff, lastin, lastout;
     float a;
  };
  
  class SimpleLP
  {
  public:
     SimpleLP() : mCutoff( 1.0f ), mSampRate( 44100.0f ), lastin( 0 ), lastout( 0 )
     {
        this->setCutoff( mCutoff );
     }
  
     inline void setSampRate( float samp )
     {
        mSampRate = samp;
     }
  
     
     void setCutoff( float cutoff )
     {
        mCutoff = cutoff / mSampRate;
        mCutoff = suba::clamp( mCutoff, 0.0f, 1.0f );
  
        float omega = atan( M_PI * mCutoff );
        a = -(1.0f - omega) / (1.0f + omega);
        b = (1.0f - b) / 2.0f;
     }
  
     inline float filter( float data )
     {
        // dest[0] = b * (src[0] + src[-1]) - a * dest[-1]
        lastout = b * (data + lastin) - a * lastout;
        lastin = data;
        return lastout;
     }
  
  private:
     float mSampRate;
     float mCutoff;
     float a, b;   // coefs
     float lastin, lastout; // sample history (data, lastin unfiltered, lastout filtered)
  };
  
  
  struct EdgeFilter {
    EdgeFilter() : mValue( false ), mValuePrev( false ), mState( UP ) {}
    enum EdgeState {
      UP=0,
      DOWN=1,
      EDGE_UP=2,
      EDGE_DOWN=3
    };
    EdgeState filter( bool value ) {
      mValuePrev = mValue;
      mValue = value;
      if (value != mValuePrev) {  
        mState = mValue ? EDGE_DOWN : EDGE_UP;
      } else {
        mState = mValue ? DOWN : UP;
      }
      return mState;
    }
    bool mValue;
    bool mValuePrev;
    EdgeState mState;
  };
  
  struct DebounceFilter {
    DebounceFilter( double timeout = 0.1 ) : mValue( false ), mValuePrev( false ), mAge( timeout ), mTimeout( timeout ), mCalls( 0 ), mCallsIgnore( 2 ) {}
    bool filter( bool value, double dt ) {
      if (mCalls < mCallsIgnore) {
        ++mCalls;
        return false;
      }
   
      // ignore any changes that aren't old enough
      if (value != mValue && mAge >= mTimeout) {
        mAge = 0;
        mValuePrev = mValue;
        mValue = value;
      }
      mAge = suba::Min( (mAge + dt), mTimeout ); // prevent wraparound
      return mValue;
    }
    bool mValue;
    bool mValuePrev;
    double mAge;
    double mTimeout;
  
    int mCalls;
    int mCallsIgnore; // matrix hardware spits out a few at the beginning, so we can ignore them...
  };
  
  // dont emit single clicks when followed by double click (introduces slight delay)
  struct SingleClickFilter {
    SingleClickFilter( double timeout = 0.5 ) : mClicks( 0 ), mAge( timeout ), mTimeout( timeout ) {}
    bool filter( EdgeFilter::EdgeState value, double dt, bool double_click_detected ) {
      if (double_click_detected) {
        mClicks = 0;
        mAge = 0.0;
      }
      else if (value == EdgeFilter::EDGE_DOWN) {
        mAge = 0.0; // reset
        ++mClicks;
      }
      if (1 == mClicks && mTimeout <= mAge) {
        mClicks = 0; // reset
        mAge = 0.0;
        return true; // we got a single click!
      }
      mAge = suba::Min( mAge + dt, mTimeout );
      return false;
    }
    int mClicks;
    double mAge;
    double mTimeout;
  };

  struct Timer {
    Timer( double timeout ) : mAge( 0.0 ), mTimeout( timeout ) {}
    bool update( double dt ) {
      mAge += dt;
      if (mAge >= mTimeout) {
        mAge = 0.0;
        return true;
      }
      return false;
    }
    void setTimeout( double timeout ) {
      mAge = 0.0;
      mTimeout = timeout;
    }
    void expireNow() {
      mAge = mTimeout;
    }
    double mAge;
    double mTimeout;
  };

}

#endif
