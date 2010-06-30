#include <math.h>
class Seabee3Util
{
public:
	template <typename _T>
	static _T angleDistRel(const _T & a1, const _T & a2) //return the distance D from @a1 to @a2 such that -180 < D < 180
	{
		_T c = a1 - a2;
		
		const _T c_p_360 ( 360 );
		const _T c_p_180 ( 180 );
		const _T c_n_360 ( -360 );
		const _T c_n_180 ( -180 );
		const _T c_zero ( 0 );
		
		//this line does the following:
		//if c < -180, subtract 360 from a2; else if c > 180, add 360 to a2; else add 0 to a2
		//return a2 - a1
		return ( a2 + ( c < c_n_180 ? c_n_360 : ( c > c_p_180 ? c_p_360 : c_zero ) ) ) - a1;
	}
	
	template <typename _T>
	static void normalizeAngle(_T & a) //force @a into 0 <= @a <= 360
	{
		//const _T c_p_360 ( 360 );
		//const int c_p_i_360 = (int) c_p_360;
		const int c_a_int = (int) a;
		const _T c_a_int_val ( c_a_int );
		const _T c_a_diff = a - c_a_int_val;
		
		const _T result (c_a_int % 360);
		
		a = result + c_a_diff;
	}
	
	template <typename _T>
	static void capValue(_T & target, const _T & magCap) //cap @target between -@magCap and @magCap
	{
		capValue(target, -magCap, magCap);
	}
	
	template <typename _T>
	static void capValue(_T & target, const _T & lowerCap, const _T & upperCap) //cap @target between @lowerCap and @upperCap
	{
		target = target < lowerCap ? lowerCap : (target > upperCap ? upperCap : target); //teh 1337
	}
	
	template <typename _T>
	static void capValueProp(_T & t1, _T & t2, const _T magCap) //scale @t1 and @t2 down by the same amount such that -@magCap <= t1, t2 <= magCap
	{
		const _T largest = fabs(t1) > fabs(t2) ? t1 : t2;
		
		if(fabs(largest) > fabs(magCap))
		{
			//_T diff = fabs(largest) - magCap;
			const double div = fabs( (double)largest ) / fabs( (double)magCap ); //explicit use of double for scaling only
			const double t1_cpy (t1);
			const double t2_cpy (t2);
			
			t1 = _T ( round( t1_cpy / div ) );
			t2 = _T ( round( t2_cpy / div ) );
		}
	}
};
