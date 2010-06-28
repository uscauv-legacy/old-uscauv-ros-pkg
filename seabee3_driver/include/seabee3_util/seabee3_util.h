class Seabee3Util
{
public:
	template <typename _T>
	static _T angleDistRel(const _T & a1, const _T & a2)
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
	static void normalizeAngle(_T & a)
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
	static void capValue(_T & target, const _T & magCap)
	{
		capValue(target, -magCap, magCap);
	}
	
	template <typename _T>
	static void capValue(_T & target, const _T & lowerCap, const _T & upperCap)
	{
		target = target < lowerCap ? lowerCap : (target > upperCap ? upperCap : target); //teh 1337
	}
};
