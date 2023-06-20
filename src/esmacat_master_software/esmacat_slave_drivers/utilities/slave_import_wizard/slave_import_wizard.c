// Esmacat slave import wizard. This code is developed by modifying SOEM library //


#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

// start of siw
#include <ctype.h>
// end of siw


char IOmap[4096];
ec_ODlistt ODlist;
ec_OElistt OElist;
boolean printSDO = FALSE;
boolean printMAP = FALSE;
char usdo[128];
char hstr[1024];

// start of siw
char pdo_input_variable_name[100][100];
char pdo_input_variable_type[100][100];
int pdo_input_variable_bit_length[100];
char pdo_output_variable_name[100][100];
char pdo_output_variable_type[100][100];
int pdo_output_variable_bit_length[100];
int pdo_input_variable_byte = 0;
int pdo_output_variable_byte = 0;
char esmacat_class_name_siw[120];
char esmacat_slave_name_for_header_file[122];
char esmacat_slave_name_for_source_file[124];
char esmacat_slave_name_without_space[100];

FILE *ofp_esmacat_slave_header_file;
FILE *ofp_esmacat_slave_source_file;

int slave_index_for_siw=0;
int esmacat_slave_input_var_index=0;
int esmacat_slave_output_var_index=0;

enum PDO_direction{PDO_input,PDO_output};
boolean ready_to_write_siw = 0;

void write_the_beginning_of_esmacat_header_file(){

    char uppercase_and_underbar_name_of_esmacat_slave[100];
    char esmacat_class_name_siw_no_prefix[100];
    // create the class name not to have a space or dot
    for(int i=0;i<100;i++) esmacat_class_name_siw_no_prefix[i] = '\0';
    for(int i = 0; ec_slave[slave_index_for_siw].name[i]; i++){
        if ( ec_slave[slave_index_for_siw].name[i] == 0x20 || ec_slave[slave_index_for_siw].name[i] == 46 ){ // if there is space (0x20) or dot (46)
            esmacat_class_name_siw_no_prefix[i] = 95;
        }
        else{
            esmacat_class_name_siw_no_prefix[i] = tolower(ec_slave[slave_index_for_siw].name[i]);
        }
    }
    sprintf(esmacat_class_name_siw,"esmacat_%s",esmacat_class_name_siw_no_prefix);
    sprintf(esmacat_slave_name_for_header_file,"%s.h",esmacat_class_name_siw);

    // convert the class name into upper_case word
    for(int i=0;i<100;i++) uppercase_and_underbar_name_of_esmacat_slave[i] = '\0';
    for(int i = 0; esmacat_slave_name_for_header_file[i]; i++){
        if ( esmacat_slave_name_for_header_file[i] == 0x20 || esmacat_slave_name_for_header_file[i] == 46 ){ // if there is space (0x20) or dot (46)
            uppercase_and_underbar_name_of_esmacat_slave[i] = 95;
        }
        else{
            uppercase_and_underbar_name_of_esmacat_slave[i] = toupper(esmacat_slave_name_for_header_file[i]);
        }
    }

    char esmacat_slave_name_of_esmacat_slave_for_vendor_product_id[100];
    for(int i=0;i<100;i++) esmacat_slave_name_of_esmacat_slave_for_vendor_product_id[i] = '\0';
    for(int i = 0; esmacat_class_name_siw[i]; i++){
        if ( esmacat_class_name_siw[i] == 0x20 || esmacat_class_name_siw[i] == 46 ){ // if there is space (0x20) or dot (46)
            esmacat_slave_name_of_esmacat_slave_for_vendor_product_id[i] = 95;
        }
        else{
            esmacat_slave_name_of_esmacat_slave_for_vendor_product_id[i] = toupper(esmacat_class_name_siw[i]);
        }
    }

    ofp_esmacat_slave_header_file = fopen (esmacat_slave_name_for_header_file, "w");
    fprintf(ofp_esmacat_slave_header_file, "#ifndef ");
    fprintf(ofp_esmacat_slave_header_file,"%s", uppercase_and_underbar_name_of_esmacat_slave);
    fprintf(ofp_esmacat_slave_header_file,"\n#define ");
    fprintf(ofp_esmacat_slave_header_file,"%s",uppercase_and_underbar_name_of_esmacat_slave);
    fprintf(ofp_esmacat_slave_header_file,"\n\n#define\t%s_VENDOR_ID\t",esmacat_slave_name_of_esmacat_slave_for_vendor_product_id);
    fprintf(ofp_esmacat_slave_header_file,"0x%08x",(int)ec_slave[slave_index_for_siw].eep_man);
    fprintf(ofp_esmacat_slave_header_file,"\n#define\t%s_PRODUCT_ID\t",esmacat_slave_name_of_esmacat_slave_for_vendor_product_id);
    fprintf(ofp_esmacat_slave_header_file,"0x%08x",(int)ec_slave[slave_index_for_siw].eep_id);
    fprintf(ofp_esmacat_slave_header_file,"\n\n#include \"esmacat_slave.h\"\n\nclass ");
    fprintf(ofp_esmacat_slave_header_file,"%s",esmacat_class_name_siw);
    fprintf(ofp_esmacat_slave_header_file,"%s", ": public esmacat_slave{ \nprivate:");
    for (int i=0;i<esmacat_slave_input_var_index ;i++){
        fprintf(ofp_esmacat_slave_header_file,"\n\t%s\t%s;", pdo_input_variable_type[i], pdo_input_variable_name[i]);
    }
    for (int i=0;i<esmacat_slave_output_var_index;i++){
        fprintf(ofp_esmacat_slave_header_file,"\n\t%s\t%s;", pdo_output_variable_type[i], pdo_output_variable_name[i]);
    }
    fprintf(ofp_esmacat_slave_header_file,"\n\npublic:");
    fprintf(ofp_esmacat_slave_header_file,"\n\t%s();", esmacat_class_name_siw);
    fprintf(ofp_esmacat_slave_header_file,"\n\tuint32_t esmacat_slave_product_id = %s_PRODUCT_ID;",esmacat_slave_name_of_esmacat_slave_for_vendor_product_id);
    fprintf(ofp_esmacat_slave_header_file,"\n\tuint32_t esmacat_slave_vendor_id = %s_VENDOR_ID;",esmacat_slave_name_of_esmacat_slave_for_vendor_product_id);
    fprintf(ofp_esmacat_slave_header_file,"\n\tuint32_t get_esmacat_product_id(){return esmacat_slave_product_id;}");
    fprintf(ofp_esmacat_slave_header_file,"\n\tuint32_t get_esmacat_vendor_id(){return esmacat_slave_vendor_id;}");
    fprintf(ofp_esmacat_slave_header_file,"\n\tvoid ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);");
    for (int i=0;i<esmacat_slave_input_var_index;i++){
        fprintf(ofp_esmacat_slave_header_file,"\n\t%s get_%s();",pdo_input_variable_type[i],pdo_input_variable_name[i]);
    }
    for (int i=0;i<esmacat_slave_output_var_index;i++){
        fprintf(ofp_esmacat_slave_header_file,"\n\tvoid set_%s(%s value);",pdo_output_variable_name[i],pdo_output_variable_type[i]);
    }
    fprintf(ofp_esmacat_slave_header_file,"\n}; \n\n#endif");
}


void write_the_beginning_of_esmacat_source_file(){

    sprintf(esmacat_slave_name_for_source_file,"%s.cpp",esmacat_class_name_siw);
    ofp_esmacat_slave_source_file= fopen (esmacat_slave_name_for_source_file, "w");

    char main_text[1000] = "";
    char Vendor_id[40];
    char Product_id[40];
    int var1, var2;

    strcat(main_text, "#include \"");
    strcat(main_text, esmacat_slave_name_for_header_file);
    strcat(main_text, "\"\n\n");
    strcat(main_text, esmacat_class_name_siw);
    strcat(main_text, "::");
    strcat(main_text, esmacat_class_name_siw);
    strcat(main_text, "(){\n");
    
    var1 = (int)ec_slave[slave_index_for_siw].eep_man;
    sprintf(Vendor_id, "%s 0x%08x;\n", "\tesmacat_slave_vendor_id =", (int)var1);
    strcat(main_text, Vendor_id);

   
    var2 = (int)ec_slave[slave_index_for_siw].eep_id;
    sprintf(Product_id, "%s 0x%08x;", "\tesmacat_slave_product_id =", (int)var2);
    strcat(main_text, Product_id);

    fprintf(ofp_esmacat_slave_source_file,"%s",main_text);

    for (int i=0;i<esmacat_slave_input_var_index ;i++){
        fprintf(ofp_esmacat_slave_source_file,"\n\t%s=0;", pdo_input_variable_name[i]);
    }

    for (int i=0;i<esmacat_slave_output_var_index;i++){
        fprintf(ofp_esmacat_slave_source_file,"\n\t%s=0;",  pdo_output_variable_name[i]);
    }
    fprintf(ofp_esmacat_slave_source_file,"\n}\n");

    for (int i=0;i<esmacat_slave_input_var_index;i++){
        fprintf(ofp_esmacat_slave_source_file,"\n%s %s::get_%s()",pdo_input_variable_type[i],esmacat_class_name_siw,pdo_input_variable_name[i]);
        fprintf(ofp_esmacat_slave_source_file,"{\n\treturn %s;  \n}",pdo_input_variable_name[i]);
    }
    for (int i=0;i<esmacat_slave_output_var_index;i++){
        fprintf(ofp_esmacat_slave_source_file,"\nvoid %s::set_%s(%s value)",esmacat_class_name_siw, pdo_output_variable_name[i],pdo_output_variable_type[i]);
        fprintf(ofp_esmacat_slave_source_file,"{\n\t%s = value;  \n}",pdo_output_variable_name[i]);
    }

    fprintf(ofp_esmacat_slave_source_file,"\nvoid %s::ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop){",esmacat_class_name_siw);

    // if the number of variable is zero or its size is less than 1 byte
    if ( pdo_input_variable_byte != 0  ){
        fprintf(ofp_esmacat_slave_source_file,"\n\tunsigned char input_variable[%d];",pdo_input_variable_byte);
    }
    else if ( pdo_input_variable_bit_length[0] != 0){
        fprintf(ofp_esmacat_slave_source_file,"\n\tunsigned char input_variable[1];");
    }

    // if the number of variable is zero or its size is less than 1 byte
    if ( pdo_output_variable_byte != 0  ){
        fprintf(ofp_esmacat_slave_source_file,"\n\tunsigned char output_variable[%d];",pdo_output_variable_byte);
    }
    else if ( pdo_output_variable_bit_length[0] !=0 ){
        fprintf(ofp_esmacat_slave_source_file,"\n\tunsigned char output_variable[1];");
    }

    for (int i = 0;i<pdo_input_variable_byte;i++){
        fprintf(ofp_esmacat_slave_source_file,"\n\tinput_variable[%d] = *(ec_slave_inputs+%d);",i,i );
    }

    int next_byte_idx = 0;
    int next_bit_idx = 0;
    for (int i = 0;i<esmacat_slave_input_var_index;i++){
        if (  pdo_input_variable_bit_length[i] < 7 ){
            fprintf(ofp_esmacat_slave_source_file,"\n\t%s = ((input_variable[%d] & (0b00000001 << %d)!= 0));",pdo_input_variable_name[i],next_byte_idx,next_bit_idx);
            next_bit_idx++;
            if (next_bit_idx >7 ){
                next_bit_idx=0;
                next_byte_idx++;
            }
        }
        else{
            int byte_of_input = pdo_input_variable_bit_length[i] / 8;
            fprintf(ofp_esmacat_slave_source_file,"\n\t%s = ",pdo_input_variable_name[i]);
            for (int j=0;j<byte_of_input;j++){
                fprintf(ofp_esmacat_slave_source_file,"+(input_variable[%d] << %d)",next_byte_idx,j*8);
                next_byte_idx++;
            }
            fprintf(ofp_esmacat_slave_source_file,";");
        }
    }
    next_byte_idx = 0;
    next_bit_idx = 0;
    for (int i = 0;i<pdo_output_variable_byte;i++){
        fprintf(ofp_esmacat_slave_source_file,"\n\toutput_variable[%d] = 0;",i );
    }
    for (int i = 0;i<esmacat_slave_output_var_index;i++){
        if (  pdo_output_variable_bit_length[i] < 7 ){
            fprintf(ofp_esmacat_slave_source_file,"\n\toutput_variable[%d] = output_variable[%d] | (%s << %d) ;",next_byte_idx,next_byte_idx, pdo_output_variable_name[i], next_bit_idx);
            next_bit_idx++;
            if (next_bit_idx >7 ){
                next_bit_idx=0;
                next_byte_idx++;
            }
        }
        else{
            int byte_of_output = pdo_output_variable_bit_length[i] / 8;
            for (int j=0;j<byte_of_output;j++){
                fprintf(ofp_esmacat_slave_source_file,"\n\toutput_variable[%d] = ",next_byte_idx);
                fprintf(ofp_esmacat_slave_source_file," +((%s >> %d) & 0x00ff);",(pdo_output_variable_name[i]),8*j);
                next_byte_idx++;
            }
        }
    }
    for (int i = 0;i<pdo_output_variable_byte;i++){
        fprintf(ofp_esmacat_slave_source_file,"\n\t*(ec_slave_outputs+%d) = output_variable[%d];",i,i );
    }
    fprintf(ofp_esmacat_slave_source_file,"\n}");
}

// end of siw


char* dtype2string(uint16 dtype)
{
    switch(dtype)
    {
    case ECT_BOOLEAN:
        sprintf(hstr, "BOOLEAN");
        break;
    case ECT_INTEGER8:
        sprintf(hstr, "INTEGER8");
        break;
    case ECT_INTEGER16:
        sprintf(hstr, "INTEGER16");
        break;
    case ECT_INTEGER32:
        sprintf(hstr, "INTEGER32");
        break;
    case ECT_INTEGER24:
        sprintf(hstr, "INTEGER24");
        break;
    case ECT_INTEGER64:
        sprintf(hstr, "INTEGER64");
        break;
    case ECT_UNSIGNED8:
        sprintf(hstr, "UNSIGNED8");
        break;
    case ECT_UNSIGNED16:
        sprintf(hstr, "UNSIGNED16");
        break;
    case ECT_UNSIGNED32:
        sprintf(hstr, "UNSIGNED32");
        break;
    case ECT_UNSIGNED24:
        sprintf(hstr, "UNSIGNED24");
        break;
    case ECT_UNSIGNED64:
        sprintf(hstr, "UNSIGNED64");
        break;
    case ECT_REAL32:
        sprintf(hstr, "REAL32");
        break;
    case ECT_REAL64:
        sprintf(hstr, "REAL64");
        break;
    case ECT_BIT1:
        sprintf(hstr, "BIT1");
        break;
    case ECT_BIT2:
        sprintf(hstr, "BIT2");
        break;
    case ECT_BIT3:
        sprintf(hstr, "BIT3");
        break;
    case ECT_BIT4:
        sprintf(hstr, "BIT4");
        break;
    case ECT_BIT5:
        sprintf(hstr, "BIT5");
        break;
    case ECT_BIT6:
        sprintf(hstr, "BIT6");
        break;
    case ECT_BIT7:
        sprintf(hstr, "BIT7");
        break;
    case ECT_BIT8:
        sprintf(hstr, "BIT8");
        break;
    case ECT_VISIBLE_STRING:
        sprintf(hstr, "VISIBLE_STRING");
        break;
    case ECT_OCTET_STRING:
        sprintf(hstr, "OCTET_STRING");
        break;
    default:
        sprintf(hstr, "Type 0x%4.4X", dtype);
    }
    return hstr;
}


char* dtype2string_for_esmacat(uint16 dtype)
{
    switch(dtype)
    {
    case ECT_BOOLEAN:
        sprintf(hstr, "bool");
        break;
    case ECT_INTEGER8:
        sprintf(hstr, "unsigned char");
        break;
    case ECT_INTEGER16:
        sprintf(hstr, "int16_t");
        break;
    case ECT_INTEGER32:
        sprintf(hstr, "int32_t");
        break;
    case ECT_INTEGER24:
        sprintf(hstr, "int24_t");
        break;
    case ECT_INTEGER64:
        sprintf(hstr, "int64_t");
        break;
    case ECT_UNSIGNED8:
        sprintf(hstr, "uint8_t");
        break;
    case ECT_UNSIGNED16:
        sprintf(hstr, "uint16_t");
        break;
    case ECT_UNSIGNED32:
        sprintf(hstr, "uint32_t");
        break;
    case ECT_UNSIGNED24:
        sprintf(hstr, "uint24_t");
        break;
    case ECT_UNSIGNED64:
        sprintf(hstr, "uint64_t");
        break;
    case ECT_REAL32:
        sprintf(hstr, "float");
        break;
    case ECT_REAL64:
        sprintf(hstr, "double");
        break;
    case ECT_BIT1:
        sprintf(hstr, "bool");
        break;
    case ECT_BIT2:
        sprintf(hstr, "bool");
        break;
    case ECT_BIT3:
        sprintf(hstr, "bool");
        break;
    case ECT_BIT4:
        sprintf(hstr, "bool");
        break;
    case ECT_BIT5:
        sprintf(hstr, "bool");
        break;
    case ECT_BIT6:
        sprintf(hstr, "bool");
        break;
    case ECT_BIT7:
        sprintf(hstr, "bool");
        break;
    case ECT_BIT8:
        sprintf(hstr, "bool");
        break;
    case ECT_VISIBLE_STRING:
        sprintf(hstr, "string");
        break;
    case ECT_OCTET_STRING:
        sprintf(hstr, "string");
        break;
    default:
        sprintf(hstr, "int");
    }
    return hstr;
}

char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype)
{
    int l = sizeof(usdo) - 1, i;
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;
    char es[32];

    memset(&usdo, 0, 128);
    ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
    if (EcatError)
    {
        return ec_elist2string();
    }
    else
    {
        switch(dtype)
        {
        case ECT_BOOLEAN:
            u8 = (uint8*) &usdo[0];
            if (*u8) sprintf(hstr, "TRUE");
            else sprintf(hstr, "FALSE");
            break;
        case ECT_INTEGER8:
            i8 = (int8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %d", *i8, *i8);
            break;
        case ECT_INTEGER16:
            i16 = (int16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %d", *i16, *i16);
            break;
        case ECT_INTEGER32:
        case ECT_INTEGER24:
            i32 = (int32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %d", *i32, *i32);
            break;
        case ECT_INTEGER64:
            i64 = (int64*) &usdo[0];
            sprintf(hstr, "0x%16.16"PRIx64" %"PRId64, *i64, *i64);
            break;
        case ECT_UNSIGNED8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%2.2x %u", *u8, *u8);
            break;
        case ECT_UNSIGNED16:
            u16 = (uint16*) &usdo[0];
            sprintf(hstr, "0x%4.4x %u", *u16, *u16);
            break;
        case ECT_UNSIGNED32:
        case ECT_UNSIGNED24:
            u32 = (uint32*) &usdo[0];
            sprintf(hstr, "0x%8.8x %u", *u32, *u32);
            break;
        case ECT_UNSIGNED64:
            u64 = (uint64*) &usdo[0];
            sprintf(hstr, "0x%16.16"PRIx64" %"PRIu64, *u64, *u64);
            break;
        case ECT_REAL32:
            sr = (float*) &usdo[0];
            sprintf(hstr, "%f", *sr);
            break;
        case ECT_REAL64:
            dr = (double*) &usdo[0];
            sprintf(hstr, "%f", *dr);
            break;
        case ECT_BIT1:
        case ECT_BIT2:
        case ECT_BIT3:
        case ECT_BIT4:
        case ECT_BIT5:
        case ECT_BIT6:
        case ECT_BIT7:
        case ECT_BIT8:
            u8 = (uint8*) &usdo[0];
            sprintf(hstr, "0x%x", *u8);
            break;
        case ECT_VISIBLE_STRING:
            strcpy(hstr, usdo);
            break;
        case ECT_OCTET_STRING:
            hstr[0] = 0x00;
            for (i = 0 ; i < l ; i++)
            {
                sprintf(es, "0x%2.2x ", usdo[i]);
                strcat( hstr, es);
            }
            break;
        default:
            sprintf(hstr, "Unknown type");
        }
        return hstr;
    }
}

/** Read PDO assign structure */
int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset, enum PDO_direction pdo_dir, boolean flag_for_siw)
{
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
    int wkc, bsize = 0, rdl;
    int32 rdat2;
    uint8 bitlen, obj_subidx;
    uint16 obj_idx;
    int abs_offset, abs_bit;

    rdl = sizeof(rdat); rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
    /* positive result from slave ? */
    if ((wkc > 0) && (rdat > 0))
    {
        /* number of available sub indexes */
        nidx = rdat;
        bsize = 0;
        /* read all PDO's */
        for (idxloop = 1; idxloop <= nidx; idxloop++)
        {
            rdl = sizeof(rdat); rdat = 0;
            /* read PDO assign */
            wkc = ec_SDOread(slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
            /* result is index of PDO */
            idx = etohl(rdat);
            if (idx > 0)
            {
                rdl = sizeof(subcnt); subcnt = 0;
                /* read number of subindexes of PDO */
                wkc = ec_SDOread(slave,idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
                subidx = subcnt;
                /* for each subindex */
                for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
                {
                    rdl = sizeof(rdat2); rdat2 = 0;
                    /* read SDO that is mapped in PDO */
                    wkc = ec_SDOread(slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
                    rdat2 = etohl(rdat2);
                    /* extract bitlength of SDO */
                    bitlen = LO_BYTE(rdat2);
                    bsize += bitlen;
                    obj_idx = (uint16)(rdat2 >> 16);
                    obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;
                    ODlist.Slave = slave;
                    ODlist.Index[0] = obj_idx;
                    OElist.Entries = 0;
                    wkc = 0;
                    /* read object entry from dictionary if not a filler (0x0000:0x00) */
                    if(obj_idx || obj_subidx)
                        wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);
                    printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);

                    if((wkc > 0) && OElist.Entries)
                    {
                        printf(" %-12s %s \n", dtype2string(OElist.DataType[obj_subidx]), OElist.Name[obj_subidx]);
                        // start of siw
                        if (flag_for_siw == 1){
                            if (pdo_dir == PDO_input){

                                sprintf(&(pdo_input_variable_name[esmacat_slave_input_var_index][0]),"input_variable_%d_%s",esmacat_slave_input_var_index,OElist.Name[obj_subidx]);
                                //                                strcpy(pdo_input_variable_name[esmacat_slave_input_var_index],temp);
                                //                                printf("%s %s",temp,pdo_input_variable_name[esmacat_slave_input_var_index]);
                                sprintf(pdo_input_variable_type[esmacat_slave_input_var_index],"%s",dtype2string_for_esmacat(OElist.DataType[obj_subidx]));
                                pdo_input_variable_bit_length[esmacat_slave_input_var_index] = bitlen;
                                esmacat_slave_input_var_index++;
                                //                                fprintf(ofp_esmacat_slave_header_file,"\n\t%s\t%s;",  pdo_input_variable_type[esmacat_slave_input_var_index],pdo_input_variable_name[esmacat_slave_input_var_index]);
                                //                                fprintf(ofp_esmacat_slave_source_file,"\n\t%s=0;", pdo_input_variable_name[esmacat_slave_input_var_index]);
                            }
                            else {

                                sprintf(pdo_output_variable_name[esmacat_slave_output_var_index],"output_variable_%d_%s",esmacat_slave_output_var_index,OElist.Name[obj_subidx]);
                                sprintf(pdo_output_variable_type[esmacat_slave_output_var_index],"%s",dtype2string_for_esmacat(OElist.DataType[obj_subidx]));
                                //                                fprintf(ofp_esmacat_slave_header_file,"\n\t%s\t%s;",  pdo_output_variable_type[esmacat_slave_output_var_index],pdo_output_variable_name[esmacat_slave_output_var_index]);
                                //                                fprintf(ofp_esmacat_slave_source_file,"\n\t%s=0;",  pdo_output_variable_name[esmacat_slave_output_var_index]);
                                pdo_output_variable_bit_length[esmacat_slave_output_var_index] = bitlen;
                                esmacat_slave_output_var_index++;
                            }
                        }
                        // end of siw
                    }
                    else
                        printf("\n");
                    bitoffset += bitlen;

                };
            };
        };
    };
    /* return total found bitlength (PDO) */
    return bsize;
}

int si_map_sdo(int slave)
{
    int wkc, rdl;
    int retVal = 0;
    uint8 nSM, iSM, tSM;
    int Tsize, outputs_bo, inputs_bo;
    uint8 SMt_bug_add;

    printf("PDO mapping according to CoE :\n");
    SMt_bug_add = 0;
    outputs_bo = 0;
    inputs_bo = 0;
    rdl = sizeof(nSM); nSM = 0;
    /* read SyncManager Communication Type object count */
    wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    if ((wkc > 0) && (nSM > 2))
    {
        /* make nSM equal to number of defined SM */
        nSM--;
        /* limit to maximum number of SM defined, if true the slave can't be configured */
        if (nSM > EC_MAXSM)
            nSM = EC_MAXSM;
        /* iterate for every SM type defined */
        for (iSM = 2 ; iSM <= nSM ; iSM++)
        {
            rdl = sizeof(tSM); tSM = 0;
            /* read SyncManager Communication Type */
            wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
            if (wkc > 0)
            {
                if((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
                {
                    SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
                    printf("Activated SM type workaround, possible incorrect mapping.\n");
                }
                if(tSM)
                    tSM += SMt_bug_add; // only add if SMt > 0

                if (tSM == 3) // outputs
                {
                    /* read the assign RXPDO */
                    printf("  SM%1d outputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].outputs - (uint8 *)&IOmap[0]), outputs_bo, PDO_output, ( (slave_index_for_siw == slave)  && ready_to_write_siw) );
                    outputs_bo += Tsize;
                }
                if (tSM == 4) // inputs
                {
                    /* read the assign TXPDO */
                    printf("  SM%1d inputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].inputs - (uint8 *)&IOmap[0]), inputs_bo, PDO_input, ( (slave_index_for_siw == slave)  && ready_to_write_siw) );
                    inputs_bo += Tsize;
                }
            }
        }
    }

    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}

int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset, enum PDO_direction pdo_dir, boolean flag_for_siw)
{
    uint16 a , w, c, e, er, Size;
    uint8 eectl;
    uint16 obj_idx;
    uint8 obj_subidx;
    uint8 obj_name;
    uint8 obj_datatype;
    uint8 bitlen;
    int totalsize;
    ec_eepromPDOt eepPDO;
    ec_eepromPDOt *PDO;
    int abs_offset, abs_bit;
    char str_name[EC_MAXNAME + 1];

    eectl = ec_slave[slave].eep_pdi;
    Size = 0;
    totalsize = 0;
    PDO = &eepPDO;
    PDO->nPDO = 0;
    PDO->Length = 0;
    PDO->Index[1] = 0;
    for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;
    if (t > 1)
        t = 1;
    PDO->Startpos = ec_siifind(slave, ECT_SII_PDO + t);
    if (PDO->Startpos > 0)
    {
        a = PDO->Startpos;
        w = ec_siigetbyte(slave, a++);
        w += (ec_siigetbyte(slave, a++) << 8);
        PDO->Length = w;
        c = 1;
        /* traverse through all PDOs */
        do
        {
            PDO->nPDO++;
            PDO->Index[PDO->nPDO] = ec_siigetbyte(slave, a++);
            PDO->Index[PDO->nPDO] += (ec_siigetbyte(slave, a++) << 8);
            PDO->BitSize[PDO->nPDO] = 0;
            c++;
            /* number of entries in PDO */
            e = ec_siigetbyte(slave, a++);
            PDO->SyncM[PDO->nPDO] = ec_siigetbyte(slave, a++);
            a++;
            obj_name = ec_siigetbyte(slave, a++);
            a += 2;
            c += 2;
            if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
            {
                str_name[0] = 0;
                if(obj_name)
                    ec_siistring(str_name, slave, obj_name);
                if (t)
                    printf("  SM%1d RXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                else
                    printf("  SM%1d TXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                printf("     addr b   index: sub bitl data_type    name\n");
                /* read all entries defined in PDO */
                for (er = 1; er <= e; er++)
                {
                    c += 4;
                    obj_idx = ec_siigetbyte(slave, a++);
                    obj_idx += (ec_siigetbyte(slave, a++) << 8);
                    obj_subidx = ec_siigetbyte(slave, a++);
                    obj_name = ec_siigetbyte(slave, a++);
                    obj_datatype = ec_siigetbyte(slave, a++);
                    bitlen = ec_siigetbyte(slave, a++);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;

                    PDO->BitSize[PDO->nPDO] += bitlen;
                    a += 2;

                    /* skip entry if filler (0x0000:0x00) */
                    if(obj_idx || obj_subidx)
                    {
                        str_name[0] = 0;
                        if(obj_name)
                            ec_siistring(str_name, slave, obj_name);

                        printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                        printf(" %-12s %s\n", dtype2string(obj_datatype), str_name);


//                        printf(" %-12s %s \n", dtype2string(OElist.DataType[obj_subidx]), OElist.Name[obj_subidx]);
                        // start of siw
                        if (flag_for_siw == 1){
                            if (pdo_dir == PDO_input){
                                sprintf(&(pdo_input_variable_name[esmacat_slave_input_var_index][0]),"input_variable_%d_%s",esmacat_slave_input_var_index,str_name);
                                //                                strcpy(pdo_input_variable_name[esmacat_slave_input_var_index],temp);
                                //                                printf("%s %s",temp,pdo_input_variable_name[esmacat_slave_input_var_index]);
                                sprintf(pdo_input_variable_type[esmacat_slave_input_var_index],"%s",dtype2string_for_esmacat(obj_datatype));
                                pdo_input_variable_bit_length[esmacat_slave_input_var_index] = bitlen;
                                esmacat_slave_input_var_index++;
                                //                                fprintf(ofp_esmacat_slave_header_file,"\n\t%s\t%s;",  pdo_input_variable_type[esmacat_slave_input_var_index],pdo_input_variable_name[esmacat_slave_input_var_index]);
                                //                                fprintf(ofp_esmacat_slave_source_file,"\n\t%s=0;", pdo_input_variable_name[esmacat_slave_input_var_index]);
                            }
                            else {
                                sprintf(pdo_output_variable_name[esmacat_slave_output_var_index],"output_variable_%d_%s",esmacat_slave_output_var_index,str_name);
                                sprintf(pdo_output_variable_type[esmacat_slave_output_var_index],"%s",dtype2string_for_esmacat(obj_datatype));
                                //                                fprintf(ofp_esmacat_slave_header_file,"\n\t%s\t%s;",  pdo_output_variable_type[esmacat_slave_output_var_index],pdo_output_variable_name[esmacat_slave_output_var_index]);
                                //                                fprintf(ofp_esmacat_slave_source_file,"\n\t%s=0;",  pdo_output_variable_name[esmacat_slave_output_var_index]);
                                pdo_output_variable_bit_length[esmacat_slave_output_var_index] = bitlen;
                                esmacat_slave_output_var_index++;
                            }
                        }
                        // end of siw

                    }
                    bitoffset += bitlen;
                    totalsize += bitlen;
                }
                PDO->SMbitsize[ PDO->SyncM[PDO->nPDO] ] += PDO->BitSize[PDO->nPDO];
                Size += PDO->BitSize[PDO->nPDO];
                c++;
            }
            else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
            {
                c += 4 * e;
                a += 8 * e;
                c++;
            }
            if (PDO->nPDO >= (EC_MAXEEPDO - 1)) c = PDO->Length; /* limit number of PDO entries in buffer */
        }
        while (c < PDO->Length);
    }
    if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return totalsize;
}


int si_map_sii(int slave)
{
    int retVal = 0;
    int Tsize, outputs_bo, inputs_bo;

    printf("PDO mapping according to SII :\n");

    outputs_bo = 0;
    inputs_bo = 0;
    /* read the assign RXPDOs */
    Tsize = si_siiPDO(slave, 1, (int)(ec_slave[slave].outputs - (uint8*)&IOmap), outputs_bo, PDO_output, ( (slave == slave_index_for_siw) && ready_to_write_siw ));
    outputs_bo += Tsize;
    /* read the assign TXPDOs */
    Tsize = si_siiPDO(slave, 0, (int)(ec_slave[slave].inputs - (uint8*)&IOmap), inputs_bo, PDO_input, ( (slave == slave_index_for_siw) && ready_to_write_siw));
    inputs_bo += Tsize;
    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}

void si_sdo(int cnt)
{
    int i, j;

    ODlist.Entries = 0;
    memset(&ODlist, 0, sizeof(ODlist));
    if( ec_readODlist(cnt, &ODlist))
    {
        printf(" CoE Object Description found, %d entries.\n",ODlist.Entries);
        for( i = 0 ; i < ODlist.Entries ; i++)
        {
            ec_readODdescription(i, &ODlist);
            while(EcatError) printf("%s", ec_elist2string());
            printf(" Index: %4.4x Datatype: %4.4x Objectcode: %2.2x Name: %s\n",
                   ODlist.Index[i], ODlist.DataType[i], ODlist.ObjectCode[i], ODlist.Name[i]);
            memset(&OElist, 0, sizeof(OElist));
            ec_readOE(i, &ODlist, &OElist);
            while(EcatError) printf("%s", ec_elist2string());
            for( j = 0 ; j < ODlist.MaxSub[i]+1 ; j++)
            {
                if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0))
                {
                    printf("  Sub: %2.2x Datatype: %4.4x Bitlength: %4.4x Obj.access: %4.4x Name: %s\n",
                           j, OElist.DataType[j], OElist.BitLength[j], OElist.ObjAccess[j], OElist.Name[j]);
                    if ((OElist.ObjAccess[j] & 0x0007))
                    {
                        printf("          Value :%s\n", SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
                    }
                }
            }
        }
    }
    else
    {
        while(EcatError) printf("%s", ec_elist2string());
    }
}

void slaveinfo(char *ifname )
{
    int cnt, i, j, nSM;
    uint16 ssigen;
    int expectedWKC;

    printf("Starting slaveinfo\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */
        if ( ec_config(FALSE, &IOmap) > 0 )
        {
            ec_configdc();
            while(EcatError) printf("%s", ec_elist2string());
            printf("%d slaves found and configured.\n",ec_slavecount);
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 3);
            if (ec_slave[0].state != EC_STATE_SAFE_OP )
            {
                printf("Not all slaves reached safe operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_SAFE_OP)
                    {
                        printf("Slave %d State=%2x StatusCode=%4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            ec_readstate();
            for( cnt = 1 ; cnt <= ec_slavecount ; cnt++)
            {
                // start of siw //
                if (cnt ==  slave_index_for_siw ){
                    pdo_input_variable_byte = ec_slave[slave_index_for_siw].Ibytes;
                    pdo_output_variable_byte = ec_slave[slave_index_for_siw].Obytes;
                    for (int j=0;j<100;j++) pdo_input_variable_bit_length[j] = 0;
                    for (int j=0;j<100;j++) pdo_output_variable_bit_length[j] = 0;
                }
                // end of siw//

                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                       cnt-1, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
                printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
                       (ec_slave[cnt].activeports & 0x02) > 0 ,
                       (ec_slave[cnt].activeports & 0x04) > 0 ,
                       (ec_slave[cnt].activeports & 0x08) > 0 );
                printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
                printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);
                for(nSM = 0 ; nSM < EC_MAXSM ; nSM++)
                {
                    if(ec_slave[cnt].SM[nSM].StartAddr > 0)
                        printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n",nSM, ec_slave[cnt].SM[nSM].StartAddr, ec_slave[cnt].SM[nSM].SMlength,
                               (int)ec_slave[cnt].SM[nSM].SMflags, ec_slave[cnt].SMtype[nSM]);
                }
                for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
                {
                    printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
                           (int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
                           ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
                           ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
                }
                printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
                       ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
                printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);
                ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
                /* SII general section */
                if (ssigen)
                {
                    ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
                    ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
                    ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
                    ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
                    if((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
                    {
                        ec_slave[cnt].blockLRW = 1;
                        ec_slave[0].blockLRW++;
                    }
                    ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
                    ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
                    ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
                }
                printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
                       ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
                printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
                       ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);
                if ((ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE) && printSDO)
                    si_sdo(cnt);
                if(printMAP)
                {
                    if (ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE)
                        si_map_sdo(cnt);
                    else
                        si_map_sii(cnt);

                }
            }
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End slaveinfo, close socket\n");
        /* stop SOEM, close socket */
        ec_close();

        // start of siw
        if (ready_to_write_siw){
            write_the_beginning_of_esmacat_header_file();
            write_the_beginning_of_esmacat_source_file();
        }

        // end of siw


    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

char ifbuf[1024];

int main()
{
   ec_adaptert * adapter = NULL;
   ec_adaptert * adapter_selected = NULL;

   printMAP = TRUE;

   int adapter_index = 0;
   int selected_adapter_index = 0;
   int i=0;
   printf("---------------------------------------------- \n");
   printf("Slaveinfo app of Esmacat Master powered by SOEM\n");
   printf("---------------------------------------------- \n");
   printf ("List of available Ethernet adapters\n");
   adapter = adapter_selected = ec_find_adapters ();
   while (adapter != NULL)
   {
       printf ("%d: %s\n", adapter_index++, adapter->name);
       adapter = adapter->next;
   }
   printf("---------------------------------------------- \n");
   if (adapter_index == 0){
       printf ("There is no available adapter" );
   }
   else{
       printf ("Please select the adapter for EtherCAT: (0 - %d): ",adapter_index-1 );
       scanf("%d",&selected_adapter_index);
   }
   for (i=0;i<selected_adapter_index;i++) adapter_selected=adapter_selected->next;
   slaveinfo(adapter_selected->name);

   printf ("\n\nPlease see the list of slaves above, then select the index of slave to create its header file and source file for interfacing with Esmacat: (0 - %d): ", ec_slavecount-1);
   int temp = 0;
   scanf("%d",&temp);
   slave_index_for_siw = temp + 1;
   ready_to_write_siw = 1;
   slaveinfo(adapter_selected->name);


   printf("\nNow the interface files of the slave you selected have been created. Copy the created source and header files to {Esmacat project folder}/esmacat-library/master. For more details, see the tutorial at https://esmacat.com \n\n\n");

   return (0);
}
