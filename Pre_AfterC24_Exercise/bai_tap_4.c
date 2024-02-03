#include<stdint.h>
#include <stdio.h>
void nhap_array(int arr[], int n)
{
    for(int i = 0; i < n; i++)
    {
        printf("arr[%d]: ",i);
        scanf("%d",&arr[i]);
    }
}

void in(int arr[], int n){
    printf("array: [ ");

    for(int i = 0; i < n; i++)
    {
        printf("%2d ", arr[i]);
    }

    printf(" ]\n");
    printf("Dia chi cua tung phan tu: \n");

     for(int i = 0; i < n; i++)
    {
        printf("arr[%d]: %2p \n",i, &arr[i]);
    }

}

void tinh_trung_binh(int arr[], int n)
{
    float tong=0;

    for(int i = 0; i < n; i++){
     tong = arr[i] + tong;
    }

    float trung_binh = tong / n;
    printf("Gia tri trung binh la: %.3f \n", trung_binh);
}

int main()
{
    int arr[100];
    int n=-1;
    while((n < 5)){
        printf("nhap gia tri n: ");
        scanf("%d", &n);
    }
    nhap_array(arr, n);
    in(arr, n);
    tinh_trung_binh(arr, n);

}