#ifndef CBIGINTEGER_H
#define CBIGINTEGER_H

#include <iostream>
#include <string.h>
#include <bitset>
#include <sstream>
#include <vector>

using namespace std;

#define UNUSED(x) (void)(x)

static uint32_t BITS[] =
{
    0x00000001, 0x00000002, 0x00000004, 0x00000008, 0x00000010, 0x00000020, 0x00000040, 0x00000080,
    0x00000100, 0x00000200, 0x00000400, 0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000,
    0x00010000, 0x00020000, 0x00040000, 0x00080000, 0x00100000, 0x00200000, 0x00400000, 0x00800000,
    0x01000000, 0x02000000, 0x04000000, 0x08000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000
};

inline uint32_t GetBit( void * src, uint32_t pos )
{
    unsigned char* ptr = ( unsigned char* ) src + pos / 8;
    return ( *ptr >> ( pos % 8 ) ) & 1;
}

inline void SetBit( void* dst, uint32_t pos, uint32_t value )
{
    unsigned char* ptr = ( unsigned char* ) dst + pos / 8;
    unsigned bit_num = pos % 8;
    *ptr = ( *ptr &~ ( 1 << bit_num ) ) | ( ( value & 1 ) << bit_num );
}

struct Flags
{
    Flags( ) : C( false ),Z( false ),V( false ),N( false ) {}
    void Clear( )
    {
        C = false;
        Z = false;
        V = false;
        N = false;
    }
    bool C, Z, V, N;
};

inline uint32_t hvAdd( uint32_t a, uint32_t b, Flags& f )
{
    uint32_t c;
    f.Clear( );

    c = a + b;
    f.N = ( c >> 31UL ) & 0x1;
    f.C = ( c < a ) && ( c < b );
    f.Z = !c;
    f.V = ( ( ( signed )a < ( signed )b ) != f.N );
    return c;
}

inline uint32_t hvSub( uint32_t a, uint32_t b, Flags& f )
{
    uint32_t c;
    f.Clear( );

    c = a - b;
    f.N = ( c >> 31UL ) & 0x1;
    f.C = ( c < a ) && ( c < b );
    f.Z = !c;
    f.V = ( ( ( signed )a < ( signed )b ) != f.N );
    return c;
}

#pragma pack(push,1)
struct CDataUnit
{
    CDataUnit( ) : m_uInt( 0 )  {}

    union
    {
         uint32_t m_uInt;
         uint16_t m_uShort[2];
         uint8_t m_uChar[4];

         struct
         {
             unsigned m_bit0 : 1; // least significant digit
             unsigned m_bit1 : 1;
             unsigned m_bit2 : 1;
             unsigned m_bit3 : 1;
             unsigned m_bit4 : 1;
             unsigned m_bit5 : 1;
             unsigned m_bit6 : 1;
             unsigned m_bit7 : 1;
             unsigned m_bit8 : 1;
             unsigned m_bit9 : 1;
             unsigned m_bit10 : 1;
             unsigned m_bit11 : 1;
             unsigned m_bit12 : 1;
             unsigned m_bit13 : 1;
             unsigned m_bit14 : 1;
             unsigned m_bit15 : 1;
             unsigned m_bit16 : 1;
             unsigned m_bit17 : 1;
             unsigned m_bit18 : 1;
             unsigned m_bit19 : 1;
             unsigned m_bit20 : 1;
             unsigned m_bit21 : 1;
             unsigned m_bit22 : 1;
             unsigned m_bit23 : 1;
             unsigned m_bit24 : 1;
             unsigned m_bit25 : 1;
             unsigned m_bit26 : 1;
             unsigned m_bit27 : 1;
             unsigned m_bit28 : 1;
             unsigned m_bit29 : 1;
             unsigned m_bit30 : 1;
             unsigned m_bit31 : 1; // most significant digit
         };
    };


    void DoTrace( )
    {
        std::cout << " as uint = " << m_uInt << std::endl;
        std::cout << " as ushort = " << m_uShort[0] << " " << m_uShort[1] << std::endl;
        std::cout << " as uchar = " << ( int )m_uChar[0] << " " << ( int )m_uChar[1] << " " << ( int )m_uChar[2] << " " << ( int )m_uChar[3] << std::endl;

        std::cout << " as bit array = " << m_bit31 << m_bit30 << m_bit29 << m_bit28 << m_bit27 << m_bit26 << m_bit25 << m_bit24 << " "
                                        << m_bit23 << m_bit22 << m_bit21 << m_bit20 << m_bit19 << m_bit18 << m_bit17 << m_bit16 << " "
                                        << m_bit15 << m_bit14 << m_bit13 << m_bit12 << m_bit11 << m_bit10 << m_bit9 << m_bit8 << " "
                                        << m_bit7 << m_bit6 << m_bit5 << m_bit4 << m_bit3 << m_bit2 << m_bit1 << m_bit0 << std::endl;
    }
};
#pragma pack(pop)

#define MAX_CDATA_SIZE 64
#define TMP_BUFFER_SIZE 256

struct CData;

inline void Mul( const CData& inDataL, const CData& inDataR, CData& outResult );
inline void Add( const CData& inDataL, const CData& inDataR, CData& outResult );
inline int32_t Cmp( const CData& inDataL, const CData& inDataR );
inline void Mod( CData& outQuotient, CData& outRemainder, const CData& inNumerator, const CData& inDenominator );
inline void Add( const CData& inDataL, const uint32_t& inDataR, CData& outResult );

void _BigIntToDecString( const CData n, std::string& s );

#pragma pack(push,1)
struct CData
{
    CData( )
    {
        memset( this, 0, sizeof( CData ) );
    }

    CData( const CData& inData )
    {
        memcpy( this, &inData, sizeof( CData ) );
    }

    CData( const std::string& inData )
    {
        std::string sData;
        bool bSigned = false;
        if( inData[0] == '-' )
        {
            bSigned = true;
            for( uint32_t i = 0; i < inData.size( ); i++ )
            {
                if( inData[i] != '-' )
                    sData.insert( sData.end( ), inData[i] );
            }
        }else
        {
            sData = inData;
            SetSigned( false );
        }

        CData ten( 10 );
        string::const_iterator iter = sData.begin( );
        string::const_iterator end_iter = sData.end( );

        for( ; iter != end_iter; ++iter )
        {
            Mul( *this, ten, *this );
            Add( *this, int( ( *iter ) - '0' ), *this );
        }
        SetSigned( bSigned );        
        DoRepack( );
    }

    CData( const int32_t& inData )
    {
        memset( this, 0, sizeof( CData ) );
        m_Data[0].m_uInt = abs( inData );

        if( inData < 0 )
            SetSigned( true );

        DoRepack( );
    }

    CDataUnit m_Data[MAX_CDATA_SIZE];
    uint32_t m_iDataSize;

    void DoRepack( )
    {
        m_iDataSize = 0;

        while( m_Data[m_iDataSize].m_uInt != 0 )
            m_iDataSize++;
    }

    uint32_t GetDataSize( ) const
    {
        return m_iDataSize;
    }

    uint32_t CalcDataSize( ) const
    {
        int m_iDataSize = 0;

        while( m_Data[m_iDataSize].m_uInt != 0 )
            m_iDataSize++;

        return m_iDataSize;
    }

private:
    bool m_bSigned;

public:
    void SetSigned( const bool& inData )
    {
        m_bSigned = inData;
    }

    bool IsSigned( ) const
    {
        return m_bSigned;
    }

    void DoTrace( )
    {
        std::stringstream ss;
        std::cout << "As bit array = " << std::endl;
        for( int i = 0; i < MAX_CDATA_SIZE; i++ )
        {
            ss << m_Data[i].m_bit31 << m_Data[i].m_bit30 << m_Data[i].m_bit29 << m_Data[i].m_bit28 << m_Data[i].m_bit27 << m_Data[i].m_bit26 << m_Data[i].m_bit25 << m_Data[i].m_bit24
               << m_Data[i].m_bit23 << m_Data[i].m_bit22 << m_Data[i].m_bit21 << m_Data[i].m_bit20 << m_Data[i].m_bit19 << m_Data[i].m_bit18 << m_Data[i].m_bit17 << m_Data[i].m_bit16
               << m_Data[i].m_bit15 << m_Data[i].m_bit14 << m_Data[i].m_bit13 << m_Data[i].m_bit12 << m_Data[i].m_bit11 << m_Data[i].m_bit10 << m_Data[i].m_bit9 <<  m_Data[i].m_bit8
               << m_Data[i].m_bit7 <<  m_Data[i].m_bit6 <<  m_Data[i].m_bit5 <<  m_Data[i].m_bit4 <<  m_Data[i].m_bit3 <<  m_Data[i].m_bit2 <<  m_Data[i].m_bit1 <<  m_Data[i].m_bit0;

            std::cout << ss.str( ) << std::endl;
            ss.str( std::string( ) );
            ss.clear( );
        }
        TraceAsDecimal( );
        std::cout << std::endl;
    }

public:
    void TraceAsDecimal( )
    {
        std::string s;
        _BigIntToDecString( *this, s );

        if( IsSigned( ) )
            s.insert( s.begin( ), '-' );

        std::cout << "As Decimal = " << s << std::endl;
    }

    void TraceAsHex( )
    {
        std::string s;
        BigIntToHexString( s );
        std::cout << "As Hex = " << s << std::endl;
    }

    void BigIntToHexString( std::string& inData )
    {
        std::stringstream ss;
        ss << "0x";

        bool fnz = false;
        for( int i = MAX_CDATA_SIZE - 1; i >= 0; i-- )
        {
            uint32_t k32 = m_Data[i].m_uInt;
            for( int j = 0; j < 8; j++ )
            {
                uint32_t k4 = ( k32 >> 28 ) & 0x0000000f;
                k32 <<= 4;
                if( fnz )
                    ss << std::hex << std::uppercase << k4;
                else
                {
                    if( k4 != 0 )
                    {
                        ss << std::hex << std::uppercase << k4;
                        fnz = true;
                    }
                }
            }
        }
        if( !fnz )
            ss << "0";

        inData = ss.str( );
    }

    void BigIntToHexStringInv( std::string& inData )
    {
        std::stringstream ss;
        ss << "0x";

        bool fnz = false;
        for( int i = 0; i < CalcDataSize(); i++ )
        {
            uint32_t k32 = m_Data[i].m_uInt;
            for( int j = 0; j < 8; j++ )
            {
                uint32_t k4 = ( k32 >> 28 ) & 0x0000000f;
                k32 <<= 4;
                if( fnz )
                    ss << std::hex << std::uppercase << k4;
                else
                {
                    if( k4 != 0 )
                    {
                        ss << std::hex << std::uppercase << k4;
                        fnz = true;
                    }
                }
            }
        }
        if( !fnz )
            ss << "0";

        inData = ss.str( );
    }

    friend ostream &operator << ( ostream &output, const CData &D )
    {
        std::string s;
        _BigIntToDecString( D, s );

        if( D.IsSigned( ) )
            s.insert( s.begin( ), '-' );

        output << s;
        return output;
    }

    CData& operator = ( const CData& a )
    {
        memcpy( this, &a, sizeof( CData ) );
        return *this;
    }
};
#pragma pack(pop)

inline void Add( const CData& inDataL, const CData& inDataR, CData& outResult );
inline CData Add( const CData& inDataL, const CData& inDataR )
{
    CData result;
    Add( inDataL, inDataR, result );
    return result;
}

inline void Sub( const CData& inDataL, const CData& inDataR, CData& outResult );
inline void Add( const CData& inDataL, const CData& inDataR, CData& outResult )
{
    if( inDataL.IsSigned( ) && !inDataR.IsSigned( ) )
    {
        CData a( inDataL );
        a.SetSigned( false );
        CData b( inDataR );
        Sub( a, b, outResult );

        if( Cmp( a, b ) > 0 )
            outResult.SetSigned( true );
         else
            outResult.SetSigned( false );
        return;
    }

    if( !inDataL.IsSigned( ) && inDataR.IsSigned( ) )
    {
        CData a( inDataL );
        CData b( inDataR );
        b.SetSigned( false );
        Sub( a, b, outResult );

        if( Cmp( a, b ) > 0 )
            outResult.SetSigned( false );
         else
            outResult.SetSigned( true );
        return;
    }

    if( inDataL.IsSigned( ) && inDataR.IsSigned( ) )
        outResult.SetSigned( true );

    uint32_t sizeFinal = inDataL.CalcDataSize( ) + inDataR.CalcDataSize( );
    uint64_t temp = 0;
    for( uint32_t i = 0; i < sizeFinal; i++ )
    {
        temp = ( uint64_t )inDataL.m_Data[i].m_uInt + ( uint64_t )inDataR.m_Data[i].m_uInt + ( temp >> 32 );
        outResult.m_Data[i].m_uInt = ( uint32_t )temp;
    }
}

inline void Add( const CData& inDataL, const uint32_t& inDataR, CData& outResult )
{
    uint64_t temp = ( uint64_t )inDataL.m_Data[0].m_uInt + ( uint64_t )inDataR;
    outResult.m_Data[0].m_uInt = ( uint32_t )temp;
}

inline void Sub( const CData& inDataL, const CData& inDataR, CData& outResult )
{
    if( inDataL.IsSigned( ) && !inDataR.IsSigned( ) )
    {
        CData a( inDataL );
        a.SetSigned( false );
        Add( a, inDataR, outResult );
        outResult.SetSigned( true );
        return;
    }

    if( !inDataL.IsSigned( ) && inDataR.IsSigned( ) )
    {
        CData b( inDataR );
        b.SetSigned( false );
        Add( inDataL, b, outResult );
        outResult.SetSigned( false );
        return;
    }

    if( Cmp( inDataL, inDataR ) < 0  )
    {
        Sub( inDataR, inDataL, outResult );
        outResult.SetSigned( true );
        return;
    }

    if( inDataL.IsSigned( ) && inDataR.IsSigned( ) )
    {
        CData tmp1( inDataL ), tmp2( inDataR );
        tmp1.SetSigned( false );
        tmp2.SetSigned( false );

        Sub( tmp1, tmp2, outResult );
        outResult.SetSigned( false );
        return;
    }

    uint8_t borrow = 0;
    uint32_t tmpA = 0;
    uint32_t tmpB = 0;

    uint32_t sizeL = inDataL.CalcDataSize( );
    uint32_t sizeR = inDataR.CalcDataSize( );
    uint32_t sizeFinal = 0;
    if( sizeL > sizeR )
        sizeFinal = sizeL;
    else
        sizeFinal = sizeR;

    for( uint32_t i = 0; i < sizeFinal; ++i )
    {
        tmpA = inDataL.m_Data[i].m_uInt;
        tmpB = inDataR.m_Data[i].m_uInt;

        if( borrow )
        {
            if( tmpA == 0 )
                tmpA = 0xFFFFFFFF;
            else
            {
                --tmpA;
                borrow = 0;
            }
        }

        if( tmpA >= tmpB )
            outResult.m_Data[i].m_uInt = tmpA - tmpB;
        else
        {
            outResult.m_Data[i].m_uInt = 0xFFFFFFFF - tmpB;
            outResult.m_Data[i].m_uInt += tmpA + 1;
            borrow = 1;
        }
    }
}

inline CData Sub( const CData& inDataL, const CData& inDataR )
{
    CData result;
    Sub( inDataL, inDataR, result );
    return result;
}

inline CData Mul( const CData& inDataL, const CData& inDataR )
{
    CData result;
    Mul( inDataL, inDataR, result );
    return result;
}

inline void Mul( const CData& inDataL, const CData& inDataR, CData& outResult )
{
    uint32_t sizeL = inDataL.CalcDataSize( );
    uint32_t sizeR = inDataR.CalcDataSize( );

    CData tmp;
    for( uint32_t j = 0; j < sizeL; ++j )
    {
        uint32_t carry_next = 0;
        for( uint32_t i = 0; i < ( sizeL + sizeR ); ++i )
        {
            Flags f;

            uint32_t accum1 = 0;
            uint32_t accum2 = 0;
            uint32_t accum3 = 0;
            uint32_t accum4 = 0;

            uint32_t j_LO = inDataL.m_Data[j].m_uInt & 0xFFFF;
            uint32_t j_HI = inDataL.m_Data[j].m_uInt >> 16;
            uint32_t i_LO = inDataR.m_Data[i].m_uInt & 0xFFFF;
            uint32_t i_HI = inDataR.m_Data[i].m_uInt >> 16;

            size_t index = i + j;

            accum1 = j_LO * i_LO;
            accum2 = j_LO * i_HI;
            accum3 = j_HI * i_LO;
            accum4 = j_HI * i_HI;

            if( carry_next )
            {
                accum1 = hvAdd( accum1, carry_next, f );
                carry_next = 0;
                if( f.C )
                    ++carry_next;
            }

            accum1 = hvAdd( accum1, ( accum2 << 16 ), f );
            if( f.C )
                ++carry_next;

            accum1 = hvAdd( accum1, ( accum3 << 16 ), f );
            if( f.C )
                ++carry_next;

            if( carry_next )
            {
                accum4 = hvAdd( accum4, carry_next, f );
                carry_next = 0;
            }

            accum4 = hvAdd( accum4, ( accum2 >> 16 ), f );
            accum4 = hvAdd( accum4, ( accum3 >> 16 ), f );

            if( index < ( sizeL + sizeR ) )
            {
                tmp.m_Data[index].m_uInt = hvAdd( tmp.m_Data[index].m_uInt, accum1, f );
                if( f.C )
                    ++carry_next;
            }
            carry_next += accum4;
        }
    }
    memcpy( &outResult, &tmp, sizeof( CData ) );

    if( inDataL.IsSigned( ) || inDataR.IsSigned( ) )
        outResult.SetSigned( true );

    if( inDataL.IsSigned( ) && inDataR.IsSigned( ) )
        outResult.SetSigned( false );
}

inline CData Shl32( const CData& inDataL, const uint32_t& iOffset )
{
    CData result;
    memset( &result, 0, sizeof( CData ) );
    for( unsigned int i = 0; i < MAX_CDATA_SIZE - iOffset; i++ )
        result.m_Data[i+iOffset].m_uInt = inDataL.m_Data[i].m_uInt;

    return result;
}

inline void Shl32( CData& inDataL, const CData& inDataR, const uint32_t& iOffset )
{
    CData result;
    memset( &result, 0, sizeof( CData ) );
    for( unsigned int i = 0; i < MAX_CDATA_SIZE - iOffset; i++ )
        result.m_Data[i+iOffset].m_uInt = inDataR.m_Data[i].m_uInt;

    memcpy( &inDataL, &result, sizeof( CData ) );
}

inline CData Shr32( const CData& inDataL, const uint32_t& iOffset )
{
    CData result;
    memset( &result, 0, sizeof( CData ) );
    for( unsigned int i = 0; i < MAX_CDATA_SIZE - iOffset; i++ )
        result.m_Data[i].m_uInt = inDataL.m_Data[i+iOffset].m_uInt;

    return result;
}

inline void Shr32( CData& inDataL, const CData& inDataR, const uint32_t& iOffset )
{
    CData result;
    memset( &result, 0, sizeof( CData ) );
    for( unsigned int i = 0; i < MAX_CDATA_SIZE - iOffset; i++ )
        result.m_Data[i].m_uInt = inDataR.m_Data[i+iOffset].m_uInt;

    memcpy( &inDataL, &result, sizeof( CData ) );
}

inline int32_t Cmp( const CData& inDataL, const CData& inDataR )
{
    // one signed
    if( inDataL.IsSigned( ) && !inDataR.IsSigned( ) )
    {
        return -1;
    }

    // one signed
    if( !inDataL.IsSigned( ) && inDataR.IsSigned( ) )
    {
        return 1;
    }

    // two unsigned
    if( !inDataL.IsSigned( ) && !inDataR.IsSigned( ) )
    {
        for( int32_t i = MAX_CDATA_SIZE - 1; i >= 0; i-- )
        {
            if( inDataL.m_Data[i].m_uInt > inDataR.m_Data[i].m_uInt )
            {
                //std::cout << "gr3 i=" << i << " inDataL.m_Data[i].m_uInt=" << inDataL.m_Data[i].m_uInt << " inDataR.m_Data[i].m_uInt=" << inDataR.m_Data[i].m_uInt << std::endl;
                return 1;
            }

            if( inDataL.m_Data[i].m_uInt < inDataR.m_Data[i].m_uInt )
            {
                return -1;
            }
        }
    }

    // two signed
    if( inDataL.IsSigned( ) && inDataR.IsSigned( ) )
    {
        for( int32_t i = MAX_CDATA_SIZE - 1; i >= 0; i-- )
        {
            if( inDataL.m_Data[i].m_uInt < inDataR.m_Data[i].m_uInt )
            {
                return 1;
            }

            if( inDataL.m_Data[i].m_uInt > inDataR.m_Data[i].m_uInt )
            {
                return -1;
            }
        }
    }
    return 0;
}

inline void Shl( CData& outResult, const CData& inData, const uint32_t& iOffset )
{
    uint32_t iDataSize = inData.CalcDataSize( );
    memcpy( &outResult, &inData, sizeof( iDataSize ) );
    unsigned move = iOffset >> 5, cut = iOffset & 31, i, j, tmp;

    iDataSize++;
    outResult.m_Data[iDataSize-1].m_uInt = outResult.m_Data[iDataSize-move-1].m_uInt << cut;

    for( i = iDataSize - 2; ( int ) i >= ( int )move; i-- )
    {
        for( j = 0, tmp = outResult.m_Data[i-move].m_uInt; j < 32 - cut; j++ )
            tmp >>= 1;
        outResult.m_Data[i+1].m_uInt = outResult.m_Data[i+1].m_uInt | tmp;
        outResult.m_Data[i].m_uInt = outResult.m_Data[i-move].m_uInt << cut;
    }
}

inline void Shr( CData& outResult, const CData& inData, const uint32_t& iOffset )
{
    UNUSED( iOffset );

    memcpy( &outResult, &inData, sizeof( CData ) );
    uint32_t lastBit = 0;
    uint32_t tmp = 0;

    uint32_t iDataSize = inData.CalcDataSize( );
    for( int i = iDataSize - 1; i >= 0; --i )
    {
        tmp = outResult.m_Data[i].m_uInt & BITS[0];
        outResult.m_Data[i].m_uInt >>= 1;
        outResult.m_Data[i].m_uInt |= ( lastBit ? BITS[31] : 0 );
        lastBit = tmp;
    }
}

inline void Abs( CData& outResult, const CData& inData )
{
    memcpy( &outResult, &inData, sizeof( CData ) );
    outResult.SetSigned( false );
}

inline void Div( CData& outQuotient, const CData& inNumerator, const CData& inDenominator )
{    
    CData Remainder;
    Mod( outQuotient, Remainder, inNumerator, inDenominator );
}

inline uint32_t GetNumBits( const CData& inData );
inline void FastMod( CData& a, const CData& b, const CData& c )
{
    if( c.IsSigned( ) )
    {
        memset( &a, 0, sizeof( CData ) );
        return;
    }

    CData mb;
    Abs( mb, b );
    if( Cmp( mb, c ) < 0 )
    {
        if( b.IsSigned( ) )
        {
            Sub( mb, c, mb );
            if( Cmp( mb, c ) == 0 )
                memset( &mb, 0, sizeof( CData ) );
        }
        memcpy( &a, &mb, sizeof( CData ) );
        a.SetSigned( false );
        return;
    }

    int32_t bblen = GetNumBits( b );
    int32_t bclen = GetNumBits( c );
    CData tc( c );
    Shl( tc, tc, bblen - bclen );

    while( Cmp( mb, c ) >= 0 )
    {
        if( Cmp( mb, tc ) >= 0 )
        {
            CData tmp;
            Sub( mb, tc, tmp );
            memcpy( &mb, &tmp, sizeof( CData ) );
        }
        Shr( tc, tc, 1 );
    }

    if( b.IsSigned( ) )
    {
        Sub( mb, c, mb );
        if( Cmp( mb, c ) == 0 )
            memset( &mb, 0, sizeof( CData ) );
    }
    memcpy( &a, &mb, sizeof( CData ) );
    a.SetSigned( false );
}

inline void NegateMod( CData& a, CData& b, const CData& c )
{
    b.SetSigned( true );
    FastMod( a, b, c );
}

inline void Mod( CData& outQuotient, CData& outRemainder, const CData& inNumerator, const CData& inDenominator )
{
    memset( &outQuotient, 0, sizeof( CData ) );
    memset( &outRemainder, 0, sizeof( CData ) );

    CData a( inNumerator );
    a.SetSigned( false );
    CData b( inDenominator );
    b.SetSigned( false );

    uint32_t sizeL = inNumerator.CalcDataSize( );
    uint32_t sizeR = inDenominator.CalcDataSize( );
    uint32_t sizeTotal = 0;

    if( sizeL > sizeR )
        sizeTotal = sizeL;
    else
        sizeTotal = sizeR;

    int iResult = 0;
    for( int i = ( sizeTotal * 32 ) - 1; i >= 0; i-- )
    {
        Shl( outRemainder, outRemainder, 1 );
        SetBit( ( void* )&outRemainder, 0, GetBit( ( void* )&a, i ) );

        iResult = Cmp( outRemainder, b );
        if( iResult == 0 || iResult == 1 )
        {
            Sub( outRemainder, b, outRemainder );
            SetBit( ( void* )&outQuotient, i, 1 );
        }
    }
}

inline void Rem( CData& r, const CData& a, const CData& b )
{
    CData q;
    CData rem;
    Mod( q, rem, a, b );

    if( a.IsSigned( ) || b.IsSigned( ) )
    {
        Add( q, CData( 1 ), q );
        q.SetSigned( true );
    }

    CData tmp;
    Mul( b, q, tmp );
    Sub( a, tmp, r );
}

inline void And( CData& outResult, const CData& inDataL, const CData& inDataR );

inline void Pow( CData& outResult, CData& a, CData& n )
{
    CData res("1");
    CData bzero("0");
    CData bone("1");

    while( Cmp(n , bzero) > 0)
    {
        CData and_res;
        CData mul_res;
        CData shr_res;

        And(and_res, n, bone);

        if( Cmp(and_res, bone) == 0 )
        {
            Mul(res, a, mul_res);
            res = mul_res;
        }

        CData a_clone = a;

        Mul(a,a_clone,mul_res);
        a = mul_res;

        Shr(shr_res,n,1);
        n = shr_res;
    }
    outResult = res;
}

inline void ModPow( CData& outResult, CData& base, CData& exp, CData& mod )
{
    CData resoult("1");
    CData bzero("0");
    CData bone("1");
    CData tmp;
    CData internal_base(base);

    while( Cmp(exp , bzero) > 0)
    {
        CData and_res;
        CData mul_res;
        CData mod_res;
        CData shr_res;

        And(and_res, exp, bone);

        if(Cmp(and_res, bone) == 0)
        {
            Mul(internal_base, resoult, mul_res);
            Mod(tmp, resoult, mul_res, mod);
        }

        CData base_clon(internal_base);
        Mul(internal_base, base_clon, mul_res);
        Mod(tmp, mod_res, mul_res, mod);
        internal_base = mod_res;

        Shr(shr_res,exp,1);
        exp = shr_res;
    }
    
    outResult = resoult;
    outResult.DoRepack();
}

inline void Pow( CData& outResult, const CData& inValue, const unsigned& inPower )
{
    CData a( inValue );
    CData r( 1 );
    unsigned y( inPower );

    for( ;; )
    {
        if( y & 1 )
            Mul( r, a, r );

        y >>= 1;
        if( y == 0 )
        {
            memcpy( &outResult, &r, sizeof( CData ) );
            return;
        }

        Mul( a, a, a );
    }
}

inline CData Pow( CData& inValue, CData& inPower )
{
    CData outResult;
    Pow( outResult, inValue, inPower );
    return outResult;
}

inline void Not( CData& inDataL, const CData& inDataR )
{
   for( int i = 0; i < MAX_CDATA_SIZE; i++ )
       inDataL.m_Data[i].m_uInt = ~inDataR.m_Data[i].m_uInt;
}

inline void And( CData& outResult, const CData& inDataL, const CData& inDataR )
{
   for( int i = 0; i < MAX_CDATA_SIZE; i++ )
       outResult.m_Data[i].m_uInt = inDataL.m_Data[i].m_uInt & inDataR.m_Data[i].m_uInt;
}

inline void Or( CData& outResult, const CData& inDataL, const CData& inDataR )
{
   for( int i = 0; i < MAX_CDATA_SIZE; i++ )
       outResult.m_Data[i].m_uInt = inDataL.m_Data[i].m_uInt | inDataR.m_Data[i].m_uInt;
}

inline void Xor( CData& outResult, const CData& inDataL, const CData& inDataR )
{
   for( int i = 0; i < MAX_CDATA_SIZE; i++ )
       outResult.m_Data[i].m_uInt = inDataL.m_Data[i].m_uInt ^ inDataR.m_Data[i].m_uInt;
}

inline bool isZero( const CData& inData )
{
    return !static_cast<bool>( inData.m_Data[0].m_uInt );
}

inline void setZero( CData& inData )
{
    memset( &inData, 0, sizeof( CData ) );
}

inline bool isSeven( const CData& inData )
{
    return ( ( inData.m_Data[0].m_uInt & 1 ) == 0 );
}

inline void setOne( CData& inData )
{
    setZero( inData );
    inData.m_Data[0].m_uInt  = 1;
}

inline bool isOdd( CData& inData )
{
    return ( ( inData.m_Data[0].m_uInt & 1 ) == 1 );
}

inline void Ebea( CData& d, CData& a, CData& b, CData& n1, CData& n2 )
{
    if( isZero( n1 ) || isZero( n2 ) )
    {
        setZero( d );
        setZero( a );
        setZero( b );
        return;
    }

    if( Cmp( n1, n2 ) > 0 )
    {
        setZero( d );
        setZero( a );
        setZero( b );
        return;
    }

    CData X( n1 );
    CData Y( n2 );
    CData g( 1 );
    while( isSeven( X ) && isSeven( Y ) )
    {
        Shr( X, X, 1 );
        Shr( Y, Y, 1 );
        Shl( g, g, 1 );
    }

    CData u( X );
    CData v( Y );
    CData A( 1 );
    CData B;
    setZero( a );
    setOne( b );

    while( !isZero( u ) )
    {
        while( isSeven( u ) )
        {
            Shr( u, u, 1 );
            if( isOdd( A ) || isOdd( B ) )
            {
                Add( A, Y, A );
                Sub( B, X, B );
            }
            Shr( A, A, 1 );
            Shr( B, B, 1 );
        }

        while( isSeven( v ) )
        {
            Shr( v, v, 1 );
            if( isOdd( a ) || isOdd( b ) )
            {
                Add( a, Y, a );
                Sub( b, X, b );
            }
            Shr( a, a, 1 );
            Shr( b, b, 1 );
        }

        if( Cmp( u, v ) >= 0 )
        {
            Sub( u, v, u );
            Sub( A, a, A );
            Sub( B, b, B );
        }else
        {
            Sub( v, u, v );
            Sub( a, A, a );
            Sub( b, B, b );
        }
    }
    Mul( g, v, d );
}

inline void AddMod( CData& x, CData& a, CData& b, CData& n )
{
    CData t;
    Add( a, b, t );
    CData q;
    Mod( q, x, t, n );
}

inline void SubMod( CData& x, CData& a, CData& b, CData& n )
{
    CData t;
    Sub( a, b, t );
    CData q;
    FastMod( x, t, n );
}

inline void MulMod( CData& x, CData& a, CData& b, CData& n )
{
    CData t;
    CData q;
    Mul( a, b, t );
    Mod( q, x, t, n );
}

inline void DivMod( CData& a, CData& b, CData& c, CData& d )
{
    CData td, ta, tb;
    Ebea( td, ta, tb, c, d );
    MulMod( a, b, ta, d );
}

inline CData invModFast( const CData& a, const CData& modulus );
inline void InvMod( CData& a, CData& b, CData& c )
{
    a = invModFast( b, c );
}

inline uint32_t GetNumBits( const CData& inData )
{
    int32_t r = inData.CalcDataSize( ) << 5;
    bool eon = false;
    for( int32_t i = ( int32_t )inData.CalcDataSize( ) - 1; ( i >= 0 ) && !eon; i-- )
    {
      uint32_t ai = inData.m_Data[i].m_uInt;
      for( int32_t j = 0; j < 32; j++ )
      {
        if( ai & 0x80000000 )
        {
            eon = true;
            break;
        }
        r--;
        ai <<= 1;
      }
    }
    return r;
}

inline CData invModFast( const CData& a, const CData& modulus )
{
    CData i = modulus;
    CData j = a;
    CData y2 = 0;
    CData y1 = 1;
    do
    {
        CData quotient;
        Div( quotient, i , j );

        CData tmp;
        Mul( j, quotient, tmp );

        CData remainder;
        Sub( i, tmp, remainder );

        CData y;
        Mul( y1, quotient, tmp );
        Sub( y2, tmp, y );

        i = j;
        j = remainder;
        y2 = y1;
        y1 = y;
    }
    while( Cmp( j, CData( 0 ) ) > 0 );

    CData tmp;
    CData mod( modulus );
    FastMod( tmp, y2, mod );
    y2 = tmp;

    if( Cmp( y2, CData( ) ) < 0 )
    {
        CData addTmp;
        Add( y2, modulus, addTmp );
        Add( addTmp, y2, y2 );
    }
    return y2;
}

inline CData Sqrt( const CData& n )
{
    CData x0 = n;
    CData x1 = n;

    Add( x1, CData( 1 ), x1 );

    CData tmp( 0 );
    Div( tmp, x1, CData( 2 ) );
    memcpy( &x1, &tmp, sizeof( CData ) );

    while( Cmp( x1, x0 ) != 0 )
    {
        x0 = x1;

        CData divTmp( 0 );
        Div( divTmp, n, x0 );
        Add( x1, divTmp, x1 );

        CData tmp;
        Div( tmp, x1, CData( 2 ) );
        memcpy( &x1, &tmp, sizeof( CData ) );
    }
    return x0;
}

inline void Sqr( CData& outResult, const CData& inData )
{
    Mul( inData, inData, outResult );
}

inline void MulAddTo( CData& x, CData& a, CData& b )
{
    CData mulRes;
    Mul( a, b, mulRes );
    Add( x, mulRes, x );
}

inline int GetRandomInt( )
{
    int max = INT32_MAX;
    int min = 0;

    srand( ( unsigned )time( NULL ) );
    return( rand( ) % ( max + 1 - min ) ) + min;
}

inline void Rnd( CData& x, const CData& bnd )
{
    CData tmp;
    Sub( bnd, CData( 1 ), tmp );
    int iLen = tmp.CalcDataSize( );

    for( int i = 0; i < iLen; i++ )
        x.m_Data[i].m_uInt = GetRandomInt( );
}

inline void __BigIntToDecString( const CData n, char chBuffer[TMP_BUFFER_SIZE], int iCurrentPos )
{
    CData r, d( 10 ), q;
    Mod( q, r, n, d );

    char chTmpBuffer[TMP_BUFFER_SIZE] = {};
    sprintf( &chTmpBuffer[0], "%d", r.m_Data[0].m_uInt );

    memcpy( &chTmpBuffer[1], &chBuffer[0], iCurrentPos );
    iCurrentPos++;
    memcpy( &chBuffer[0], &chTmpBuffer[0], strlen( chTmpBuffer ) );

    if( Cmp( q, CData( ) ) == 0 )
        return;

    __BigIntToDecString( q, chBuffer, iCurrentPos );
}

inline void _BigIntToDecString( const CData n, std::string& s )
{
    CData r, d, q;
    d.m_Data[0].m_uInt = 10;

    Mod( q, r, n, d );

    std::stringstream ss;
    ss << r.m_Data[0].m_uInt;
    s.insert( 0, ss.str( ) );

    if( Cmp( q, CData( ) ) == 0 )
        return;

    _BigIntToDecString( q, s );
}

inline void FromBytesArray( CData& outData, const unsigned char* p, const uint32_t& n )
{
    std::vector<unsigned char> vTmp;
    vTmp.resize(n);
    memcpy( &vTmp[0], p, n);
    std::vector<unsigned char> vTmp1(vTmp.rbegin(), vTmp.rend());
    memcpy( &outData.m_Data[0].m_uInt, &vTmp1[0], n  );

    for( int i = 0; i < MAX_CDATA_SIZE; i++ )
    {
       // std::swap( outData.m_Data[i].m_uChar[3], outData.m_Data[i].m_uChar[0] );
       // std::swap( outData.m_Data[i].m_uChar[1], outData.m_Data[i].m_uChar[2] );
    }
    outData.DoRepack();
}

inline void ToBytesArray( unsigned char *p, const CData& a, long n )
{
    unsigned char* tmp = new unsigned char[n];
    memcpy( tmp, &a.m_Data[0].m_uInt, n );
    for(int i=0;i<n;i++)
        p[n-i-1] = tmp[i];
    delete[] tmp;
}

inline std::string HexToDec( const std::string& inData )
{
    CData result;
    char chTempBuffer = 0;
    for( unsigned int i = 0; i < inData.size( ); i++ )
    {
        switch( ( int )inData[i] )
        {
            // F
            case 70:
            case 102:
                chTempBuffer = 15;
                break;

            // E
            case 69:
            case 101:
                chTempBuffer = 14;
                break;

            // D
            case 68:
            case 100:
                chTempBuffer = 13;
                break;

            // C
            case 67:
            case 99:
                chTempBuffer = 12;
                break;

            // B
            case 66:
            case 98:
                chTempBuffer = 11;
                break;

            // A
            case 65:
            case 97:
                 chTempBuffer = 10;
                 break;

            default:
            chTempBuffer = inData[i] - 48;
                break;
        }

        CData powerRes;
        Pow( powerRes, CData( 16 ), ( unsigned int )( ( inData.size( ) - 1 ) - i )  );

        CData mulRes;
        Mul( CData( ( int )chTempBuffer ), powerRes, mulRes );
        Add( result,  mulRes, result );
    }
    std::stringstream ss;
    ss << result;
    return ss.str( );
}

#endif
