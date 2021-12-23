#include <iostream>
#include "CBigInteger.h"
#include <openssl/rsa.h>


bool RSA_key_gen(const uint32_t key_bits, std::string& modulus_str, std::string& pub_str, std::string& priv_str)
{
    unsigned char key_buf[256];
    if(key_bits/CHAR_BIT > sizeof(key_buf))
        return false;

    RSA             *r = RSA_new();
    BIGNUM          *bne = BN_new();
    unsigned long   e = RSA_F4; // 0x10001 

//--------generate rsa keys-------------------------------------------------------------
    int ret = BN_set_word(bne,e);
    if(ret != 1){
        RSA_free(r);
        BN_free(bne);
        return false;
    }

    ret = RSA_generate_key_ex(r, key_bits, bne, NULL);
    if(ret != 1){
        RSA_free(r);
        BN_free(bne);
        return false;
    }
//--------keys to BIGNUM----------------------------------------------------------------
    const BIGNUM *bmodulus, *bpub, *bpriv;
    RSA_get0_key(r, &bmodulus, &bpub, &bpriv);

//--------keys to std::string-----------------------------------------------------------
    int bn_size = BN_bn2bin(bmodulus, &key_buf[0]);
    if(bn_size != 0)
        modulus_str = std::string((const char*)&key_buf[0], bn_size);
    else
        return false;

    bn_size = BN_bn2bin(bpub, &key_buf[0]);
    if(bn_size != 0)
        pub_str = std::string((const char*)&key_buf[0], bn_size);
    else
        return false;

    bn_size = BN_bn2bin(bpriv, &key_buf[0]);
    if(bn_size != 0)
        priv_str = std::string((const char*)&key_buf[0], bn_size);
    else
        return false;

    RSA_free(r);
    BN_free(bne);
    return true;
}

void printStrAsHexByte(std::string label, std::string str)
{
    std::cout << label << " length = " << std::dec << str.length() << std::endl;
    for(int i=0; i< str.length(); i++)
    {
        if(i!=0 && i%16 == 0)
            std::cout << std::endl;   
        int hex = str[i]&0xff;     
        std::cout << " 0x" << std::hex << hex;
    }   
    std::cout << std::endl;
}

bool RSA_public_encrypt(const std::string& message_str, const std::string& e_str, const std::string& modulus_str, std::string& ciphertext_str)
{
    unsigned char ciphertext_buf[256];
    if(sizeof(ciphertext_buf) < modulus_str.size())
        return false;

    CData e, message, modulus, ciphertext;

    FromBytesArray( message, (const unsigned char*)message_str.c_str(), message_str.length());
    FromBytesArray( modulus, (const unsigned char*)modulus_str.c_str(), modulus_str.length());
    FromBytesArray( e, (const unsigned char*)e_str.c_str(), e_str.length());

/*  
    message.TraceAsHex();
    modulus.TraceAsHex();
    e.TraceAsHex();
*/

    // encrypt
    ModPow( ciphertext, message, e, modulus);

    ToBytesArray( ciphertext_buf, ciphertext, modulus_str.size());
    ciphertext_str = std::string((const char*)ciphertext_buf, modulus_str.size());

    return true;
}

bool RSA_private_decrypt(const std::string& ciphertext_str, const std::string& d_str, const std::string& modulus_str, std::string& message_str)
{
    unsigned char message_buf[256];
    if(sizeof(message_buf) < modulus_str.size())
        return false;

    CData ciphertext, d, message, modulus;

    FromBytesArray( ciphertext, (const unsigned char*)ciphertext_str.c_str(), ciphertext_str.length());
    FromBytesArray( modulus, (const unsigned char*)modulus_str.c_str(), modulus_str.length());
    FromBytesArray( d, (const unsigned char*)d_str.c_str(), d_str.length());

/*  
    ciphertext.TraceAsHex();
    modulus.TraceAsHex();
    d.TraceAsHex();
*/
 
    // decrypt
    ModPow(message, ciphertext, d, modulus );
    uint32_t message_size = message.m_iDataSize*4;
    ToBytesArray( message_buf, message, message_size);
    message_str = std::string((const char*)message_buf, message_size);

    return true;
}

bool myTest()
{
    std::string e_str, d_str, modulus_str, ciphertext_str, message2_str;
    uint32_t key_bits = 256;

    bool result = RSA_key_gen(key_bits, modulus_str, e_str, d_str);
    if(result == false)
        return result;

    const unsigned char message[] = {0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,
                                     0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};
    std::string message_str = std::string((const char*)&message[0], sizeof(message));

    printStrAsHexByte("modulus_str", modulus_str);
    printStrAsHexByte("e_str", e_str);
    printStrAsHexByte("d_str", d_str);
    printStrAsHexByte("message_str", message_str);

    result = RSA_public_encrypt(message_str, e_str, modulus_str, ciphertext_str);
    if(result == false)
        return result;

    printStrAsHexByte("ciphertext_str", ciphertext_str);

    result = RSA_private_decrypt(ciphertext_str, d_str, modulus_str, message2_str);
    if(result == false)
        return result;

    printStrAsHexByte("cmessage2_str", message2_str);

    if(message_str == message2_str)
        result = true;

    return result;
}

int main(int, char**) {
    std::cout << "Hello, RSA!\n";
    bool res = myTest();
    if(res == true)
        std::cout << "Test OK!" << std::endl;
    else
        std::cout << "Test OK!" << std::endl;
}

