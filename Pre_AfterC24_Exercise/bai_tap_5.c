#include<stdint.h>
#include <stdio.h>
#include<string.h>
#include<math.h>

typedef struct
{
char name[32];
char mssv[8];
uint32_t course_c;
} infor ;

 void infor_input (infor *HS)
 {
    printf("nhap ten: ");
    gets(HS->name);
    printf("nhap MSSV: ");
    gets(HS->mssv);
    printf("nhap khoa hoc: ");
    scanf("%d",&HS->course_c);
}

void infor_print(infor HS)
{
    printf("Ho va ten la: %s\n",HS.name);
    printf("MSSV la: %s\n",HS.mssv);
    printf("Khoa hoc la: %d\n",HS.course_c);
}

int main()
{
    infor HS;
    infor_input(&HS);
    infor_print(HS);
}