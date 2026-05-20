/*
 * このファイルは tecsgen により自動生成されました
 * このファイルを編集して使用することは、意図されていません
 */
#ifndef sSIOAsyncCBR_TECSGEN_H
#define sSIOAsyncCBR_TECSGEN_H

/*
 * signature   :  sSIOAsyncCBR
 * global name :  sSIOAsyncCBR
 * context     :  task
 */

#ifndef TOPPERS_MACRO_ONLY

/* シグニチャディスクリプタ #_SD_# */
struct tag_sSIOAsyncCBR_VDES {
    struct tag_sSIOAsyncCBR_VMT *VMT;
};

/* シグニチャ関数テーブル #_SFT_# */
struct tag_sSIOAsyncCBR_VMT {
    ER_UINT        (*popSend__T)( const struct tag_sSIOAsyncCBR_VDES *edp, char* dst );
    ER_UINT        (*pushReceive__T)( const struct tag_sSIOAsyncCBR_VDES *edp, char src );
    ER_UINT        (*sizeSend__T)( const struct tag_sSIOAsyncCBR_VDES *edp );
};

/* シグニチャディスクリプタ(動的結合用) #_SDES_# */
#ifndef Descriptor_of_sSIOAsyncCBR_Defined
#define  Descriptor_of_sSIOAsyncCBR_Defined
typedef struct { struct tag_sSIOAsyncCBR_VDES *vdes; } Descriptor( sSIOAsyncCBR );
#endif
#endif /* TOPPERS_MACRO_ONLY */

/* function id */
#define	FUNCID_SSIOASYNCCBR_POPSEND            (1)
#define	FUNCID_SSIOASYNCCBR_PUSHRECEIVE        (2)
#define	FUNCID_SSIOASYNCCBR_SIZESEND           (3)

#endif /* sSIOAsyncCBR_TECSGEN_H */
