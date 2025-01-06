
#define TRUE 1
#define FALSE 0
#define SIZE 18

const char cstr1[] = "Constant String1";

static int mstatic1, mstatic2;

float vfloat = 1.0;
double vdouble = 1.0;

typedef int tdef1;

tdef1 vtdef1;

typedef short tdef2[10];

tdef2 vtdef2;

struct
{
	int x, y;
}
vstruct1;

char vchar;
short vshort;
int vint;
long vlong;

char * vpchar;
short * vpshort;
int * vpint;
long * vplong;

unsigned char vuchar;
unsigned short vushort;
unsigned int vuint;
unsigned long vulong;

unsigned char * vpuchar;
unsigned short * vpushort;
unsigned int * vpuint;
unsigned long * vpulong;

unsigned char * * vppuchar;
unsigned short * * vppushort;
unsigned int * * vppuint;
unsigned long * * vppulong;

unsigned long * * * vpppulong;
unsigned long * * * * vppppulong;

int varray[10];

typedef int tarray[10];

tarray vtarray;

char vdarray[5];

char vdiarray[] = "abcd";

char vdblarray[5][6];

char vtripplearray[2][3][4];

struct struct4 {
		struct struct5 * pstruct5;
		struct struct5 * * ppstruct5;
		struct struct5 * pastruct5[2];
		} ;

struct struct5 {
		struct struct4 * pstruct4;
		struct struct4 * * ppstruct4;
		struct struct4 * pastruct4[2];
		} ;


typedef struct struct1 {
	char * word;
	int count;
	struct struct1 * left;
	struct struct1 * right;
	int field1:2;
	unsigned field2:3;
	} strtype1 ;

strtype1 ast;

typedef struct struct1 structarray[10];

structarray stra1;

struct struct4 stra2[5][5];

struct union1 {
	char * word;
	int count;
	struct struct1 * left;
	struct struct1 * right;
	union {
		int ival;
		float fval;
		char * pval;
		} uval;
	int field1:2;
	unsigned field2:3;
} aun;

struct struct2 {
	char * word;
	int count;
	char name[10];
} str2;

struct struct6 {
	int x;
	struct struct7 {
		char * word;
		int count;
		} vstruct7;
	int y;
} str6;


struct abc
{
	int x;
	int y;
};

struct abc def;

union tunion
{
	int z1;
	short z2;
	char z3;
	struct abc z4;
} vunion;

enum enumtyp { enum1, enum2, enum4=4, enum7=7, enum8, enumx= -1 } enumvar;

int ( *funcptr ) ();

func0()					/* empty function */
{
}

static void func1( intptr )		/* static function */
int * intptr;
{
	(*intptr)++;
}

void func2()
{
	int autovar;
	register int regvar;
	static int fstatic = 44;	/* initialized static variable */
	static int fstatic2;		/* not initialized static variable */

	autovar = regvar = fstatic;
	autovar++;

	func1( &autovar );	/* to force autovar as stack-scope */

	func1( &fstatic );	/* to force fstatic as static-scope */

	for ( regvar = 0; regvar < 5 ; regvar++ )
		mstatic1 += regvar*autovar;

	fstatic += mstatic1;

	fstatic2 = 2*fstatic;

	func1( &fstatic2 );
}

void func2a()
{
	auto char autovar;	/* char stack variable */
	register char regvar;	/* char register variable */

	autovar = regvar = mstatic1;
	autovar++;

	for ( regvar = 0; regvar < (char) 5 ; regvar++ )
		vchar += regvar*autovar;
}

void func2b()
{
	auto long autovar;	/* long stack variable */
	register long regvar;	/* long register variable */

	autovar = regvar = mstatic1;
	autovar++;

	for ( regvar = 0; regvar < 5l ; regvar++ )
		vlong += regvar*autovar;
}

void func2c()
{
	auto double autovar;	/* double stack variable */
	register double regvar;	/* double register variable */

	autovar = regvar = (double) mstatic1;
	autovar++;

	for ( regvar = 0.0; regvar < 5.0 ; regvar += 1.0 )
		vdouble += regvar*autovar;
}

void func2d()
{
	auto short autovar;	/* short stack variable */
	register short regvar;	/* short register variable */

	autovar = regvar = mstatic1;
	autovar++;

	for ( regvar = 0; regvar < 5l ; regvar++ )
		vlong += regvar*autovar;
}


static int func3()			/* simple function */
{
	return 5;
}

struct struct1 func4( str )		/* function returning struct */
struct struct1 str;
{
	str.count++;

	return str;
}

int func5( a, b, c )			/* multiple arguments */
int a;
char b;
long c;
{
	return a+b*c;
}

float func6( a, b )
float a;
float b;
{
	vfloat = 1.0;
	vfloat = -1.0;
	vfloat = 10.0;
	vfloat = 1.6;

	return a*b;
}

double func7( a, b )
double a;
double b;
{
	vdouble = 1.0;
	vdouble = -1.0;
	vdouble = 10.0;
	vdouble = 1.6;

	return a*b;
}

struct bfield
{
	int a:1;
	int b:2;
	int c:3;
	int d:7;
	int e:8;
	int f:15;
	int g:1;
	int h:15;
	int i:1;
	int j:16;
	unsigned k:1;
	unsigned l:2;
	unsigned m:16;
};

struct bfield vbfield;

void func8()
{
	vbfield.a = 1;
	vbfield.b = 1;
	vbfield.c = 1;
	vbfield.d = 1;
	vbfield.e = 1;
	vbfield.f = 1;
	vbfield.g = 1;
	vbfield.h = 1;
	vbfield.i = 1;
	vbfield.j = 1;
	vbfield.k = 1;
	vbfield.l = 1;
	vbfield.m = 1;

	vbfield.a = -1;
	vbfield.b = -1;
	vbfield.c = -1;
	vbfield.d = -1;
	vbfield.e = -1;
	vbfield.f = -1;
	vbfield.g = -1;
	vbfield.h = -1;
	vbfield.i = -1;
	vbfield.j = -1;
	vbfield.k = -1;
	vbfield.l = -1;
	vbfield.m = -1;
}

static int * func9()			/* nested local variables */
{
	static stat1 = 0;
	register reg1;
	auto auto1;

	auto1 = stat1;

	for ( reg1 = 0 ; reg1 < 2 ; reg1++ )
	{
		static stat2 = 0;
		register reg2;
		auto auto2;

		auto2 = stat2;

		for ( reg2 = 0 ; reg2 < reg1 ; reg2++ )
		{
			func1( &stat1 );
			func1( &auto1 );
			func1( &stat2 );
			func1( &auto2 );
		}
	}

	return &stat1;
}

func10()
{
	register i, j;
	register v1, v2, v3, v4, v5, v6, v7, v8;
	register v9, v10, v11, v12, v13, v14, v15, v16, v17;

	v17 = 0;
	for ( i = 0 ; i < 3 ; i++ )
		v17 += i;

	for ( v1 = 0 ; v1 < 3 ; v1++ )
		v17 += v1;
	for ( v2 = 0 ; v2 < 3 ; v2++ )
		v17 += v2;
	for ( v3 = 0 ; v3 < 3 ; v3++ )
		v17 += v3;
	for ( v4 = 0 ; v4 < 3 ; v4++ )
		v17 += v4;
	for ( v5 = 0 ; v5 < 3 ; v5++ )
		v17 += v5;
	for ( v6 = 0 ; v6 < 3 ; v6++ )
		v17 += v6;
	for ( v7 = 0 ; v7 < 3 ; v7++ )
		v17 += v7;
	for ( v8 = 0 ; v8 < 3 ; v8++ )
		v17 += v8;
	for ( v9 = 0 ; v9 < 3 ; v9++ )
		v17 += v9;
	for ( v10 = 0 ; v10 < 3 ; v10++ )
		v17 += v10;
	for ( v11 = 0 ; v11 < 3 ; v11++ )
		v17 += v11;
	for ( v12 = 0 ; v12 < 3 ; v12++ )
		v17 += v12;
	for ( v13 = 0 ; v13 < 3 ; v13++ )
		v17 += v13;
	for ( v14 = 0 ; v14 < 3 ; v14++ )
		v17 += v14;
	for ( v15 = 0 ; v15 < 3 ; v15++ )
		v17 += v15;
	for ( v16 = 0 ; v16 < 3 ; v16++ )
		v17 += v16;

	i = j = v17;

	for ( v1 = 0 ; v1 < 3 ; v1++ )
		v17 += v1;
	for ( v2 = 0 ; v2 < 3 ; v2++ )
		v17 += v2;
	for ( v3 = 0 ; v3 < 3 ; v3++ )
		v17 += v3;
	for ( v4 = 0 ; v4 < 3 ; v4++ )
		v17 += v4;
	for ( v5 = 0 ; v5 < 3 ; v5++ )
		v17 += v5;
	for ( v6 = 0 ; v6 < 3 ; v6++ )
		v17 += v6;
	for ( v7 = 0 ; v7 < 3 ; v7++ )
		v17 += v7;
	for ( v8 = 0 ; v8 < 3 ; v8++ )
		v17 += v8;
	for ( v9 = 0 ; v9 < 3 ; v9++ )
		v17 += v9;
	for ( v10 = 0 ; v10 < 3 ; v10++ )
		v17 += v10;
	for ( v11 = 0 ; v11 < 3 ; v11++ )
		v17 += v11;
	for ( v12 = 0 ; v12 < 3 ; v12++ )
		v17 += v12;
	for ( v13 = 0 ; v13 < 3 ; v13++ )
		v17 += v13;
	for ( v14 = 0 ; v14 < 3 ; v14++ )
		v17 += v14;
	for ( v15 = 0 ; v15 < 3 ; v15++ )
		v17 += v15;
	for ( v16 = 0 ; v16 < 3 ; v16++ )
		v17 += v16;

	return v17;
}


int func11( x )				/* multiple returns */
int x;
{
	switch ( x )
	{
	case 1:
		x = x+1;
		x = x*2;
		return x*x;
	case 2:
		return x+x;
	case 3:
		return x-x;
	case 4:
		x = x+1;
		x = x*2;
		return x*x;
	case 5:
		break;
	case 6:
		return x+x;
	default:
		break;
	}
	return x;
}


int func13( a, c, e )			/* arguments and locals stack-tracking */
int a, c, e;
{
	int b, d, f;

	b = a+c+e;
	f = b+a;
	d = f*b;

	if ( e > 0 )
		c += func13( b, f, e-1 );

	return c+e+d;
}

int func14( x1 )		/* Parameter: 1 Byte */
char x1;
{
	return 2*x1;
}

int func15( x1 )		/* Parameter: 1 Word */
short x1;
{
	return 2*x1;
}

int func16( x1 )		/* Parameter: 1 Long */
long x1;
{
	return (int) (2*x1);
}

int func17( x1, x2 )		/* Parameter: 2 Short */
short x1, x2;
{
	return x1*x2;
}

int func18( x1, x2 )		/* Parameter: Short,Long */
short x1;
long x2;
{
	return x1*x2;
}

int func19( x1, x2 )		/* Parameter: Long,Short */
long x1;
short x2;
{
	return x1*x2;
}

int func20( x1, x2, x3 )	/* Parameter: 3 Short */
short x1, x2, x3;
{
	return x1*x2*x3;
}

int func21( x1, x2, x3 )	/* Parameter: Long,Short,Short */
long x1;
short x2, x3;
{
	return x1*x2*x3;
}

int func22( x1, x2, x3 )	/* Parameter: Short,Long,Short */
short x1;
long x2;
short x3;
{
	return x1*x2*x3;
}

int func23( x1, x2, x3 )	/* Parameter: Short,Short,Long */
short x1, x2;
long x3;
{
	return x1*x2*x3;
}

char func24()			/* Char Return */
{
	return 55;
}

long func25()			/* Long Return */
{
	return 12345678l;
}

char * func26()			/* Pointer Return */
{
	static char x1[] = "abc";

	return x1;
}

struct struct1 func27( x1, x2 )		/* function returning struct */
short x1, x2;
{
	ast.count = x1*x2;

	return ast;
}

double sinewave[630];

extern double sin();

void func40()
{
	int index;
	double x;

	index = 0;
	for ( x = 0.0 ; x < 62.8 ; x += 0.1 )
		sinewave[index++] = sin(x)/(x+0.1);
}

main()
{
        int j;
	char * p;

	vtripplearray[0][0][0] = 1;
	vtripplearray[1][0][0] = 2;
	vtripplearray[0][1][0] = 3;
	vtripplearray[0][0][1] = 4;

	func2();

	func2a();

	func2b();

	func2c();

	func2d();

	funcptr = func3;

	ast.count = 12345;
	ast.left = &ast;
	ast.field1 = 1;
	ast.field2 = 2;

	ast = func4( ast );

	j = (*funcptr)();

start:
	j = func5( (int) j, (char) 2, (long) 3 );

	if ( j == 0 )
		goto start;

	vfloat = 2.0;

	func6( vfloat, (float) 3.0 );

	vdouble = 2.0;

	func7( vdouble, (double) 3.0 );

	func8();

	func9();

	func10();

	func11( 5 );

	func13( 1, 2, 3 );

	func14( (char) 55 );

	func15( (short) 55 );

	func16( (long) 55 );

	func17( (short) 44, (short) 55 );

	func18( (short) 44, (long) 55 );

	func19( (long) 44, (short) 55 );

	func20( (short) 33, (short) 44, (short) 55 );

	func21( (long) 33, (short) 44, (short) 55 );

	func22( (short) 33, (long) 44, (short) 55 );

	func23( (short) 33, (short) 44, (long) 55 );

	j = func24();

	j = func25();

	p = func26();

	func40();

	while ( TRUE )
	{
       	        sieve();
	}
}


char flags[SIZE+1];

int sieve()				/* sieve of erathostenes */
{
        register int i, primz, k;
	int anzahl;

	anzahl = 0;

        for ( i = 0 ; i <= SIZE ; flags[ i++ ] = TRUE ) ;

        for ( i = 0 ; i <= SIZE ; i++ )
        {
                if ( flags[ i ] )
                {
                        primz = i + i + 3;
                        k = i + primz;
                        while ( k <= SIZE )
                        {
                                flags[ k ] = FALSE;
                                k += primz;
                        }
                        anzahl++;
                }
        }

	return anzahl;
}

int background()			/* job for background-demo */
{
	register long count1, count2;

	count1 = count2 = 0;

	while ( TRUE )
	{
		while ( ++count1 ) ;
		count2++;
	}

	return TRUE;
}
