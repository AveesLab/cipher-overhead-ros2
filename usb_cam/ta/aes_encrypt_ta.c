#include <tee_internal_api.h>
#include <mbedtls/aes.h>
#include <string.h>
#include "ta_aes.h"

#define MAX_KEY_SIZE 32  // 최대 256비트 키
#define IV_SIZE 16       // AES CBC 모드의 16바이트 IV

// 'A'로 채워진 키와 IV를 초기화
static const uint8_t default_key[MAX_KEY_SIZE] = { [0 ... MAX_KEY_SIZE-1] = 'A' };
static const uint8_t default_iv[IV_SIZE] = { [0 ... IV_SIZE-1] = 'A' };

TEE_Result aes_encrypt(const char *plaintext, size_t plaintext_len, char *ciphertext, size_t *ciphertext_len, size_t key_len) {
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    // AES 키 설정
    if (mbedtls_aes_setkey_enc(&aes, default_key, key_len * 8) != 0) {  // key_len은 바이트 단위로 전달됨
        mbedtls_aes_free(&aes);
        return TEE_ERROR_GENERIC;
    }

    // AES CBC 모드 암호화 수행
    if (mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, plaintext_len, default_iv, (const unsigned char *)plaintext, (unsigned char *)ciphertext) != 0) {
        mbedtls_aes_free(&aes);
        return TEE_ERROR_GENERIC;
    }

    *ciphertext_len = plaintext_len;
    mbedtls_aes_free(&aes);
    return TEE_SUCCESS;
}

TEE_Result aes_decrypt(const char *ciphertext, size_t ciphertext_len, char *plaintext, size_t *plaintext_len, size_t key_len) {
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    // AES 키 설정
    if (mbedtls_aes_setkey_dec(&aes, default_key, key_len * 8) != 0) {
        mbedtls_aes_free(&aes);
        return TEE_ERROR_GENERIC;
    }

    // AES CBC 모드 복호화 수행
    if (mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, ciphertext_len, default_iv, (const unsigned char *)ciphertext, (unsigned char *)plaintext) != 0) {
        mbedtls_aes_free(&aes);
        return TEE_ERROR_GENERIC;
    }

    *plaintext_len = ciphertext_len;
    mbedtls_aes_free(&aes);
    return TEE_SUCCESS;
}

TEE_Result TA_InvokeCommandEntryPoint(void *sess_ctx, uint32_t cmd_id, uint32_t param_types, TEE_Param params[4]) {
    (void)sess_ctx;

    if (param_types != TEE_PARAM_TYPES(TEE_PARAM_TYPE_MEMREF_INPUT, TEE_PARAM_TYPE_MEMREF_OUTPUT, TEE_PARAM_TYPE_VALUE_INPUT, TEE_PARAM_TYPE_NONE)) {
        return TEE_ERROR_BAD_PARAMETERS;
    }

    // 입력된 평문과 출력 암호문, 키 길이 참조
    const char *input_data = (char *)params[0].memref.buffer;
    size_t input_len = params[0].memref.size;
    char *output_data = (char *)params[1].memref.buffer;
    size_t *output_len = &params[1].memref.size;
    size_t key_len = params[2].value.a;

    // key_len이 16, 24, 32 바이트(128, 192, 256 비트)인지 확인
    if (key_len != 16 && key_len != 24 && key_len != 32) {
        return TEE_ERROR_BAD_PARAMETERS;
    }

    if (cmd_id == TA_AES_CMD_ENCRYPT) {
        return aes_encrypt(input_data, input_len, output_data, output_len, key_len);
    } else if (cmd_id == TA_AES_CMD_DECRYPT) {
        return aes_decrypt(input_data, input_len, output_data, output_len, key_len);
    } else {
        return TEE_ERROR_NOT_SUPPORTED;
    }
}

