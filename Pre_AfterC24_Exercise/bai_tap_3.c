#include<stdint.h>
#include <stdio.h>
void nhap_array(int arr[], int n)
{
    /*Bắt đầu gắn giá trị cho mảng*/
    for(int i = 0;i < n; i++)
    {
        printf("arr[%d]: ", i);
        scanf("%d", &arr[i]);
    }
    /*Kết thúc gắn giá trị cho mảng*/
}
void in(int arr[], int n)
{
    printf("array: [ ");
    /*Bắt đầu in giá trị của mảng*/
    for(int i = 0; i < n; i++)
    {
        printf("%2d ", arr[i]);
    }
    /*Kết thúc i giá trị của mảng*/
    printf(" ]\n");
    printf("Dia chi cua tung phan tu: \n");
    /*Bat dau in dia chi*/
    for(int i = 0; i < n; i++)
    {
        printf("arr[%d]: %2p \n", i, &arr[i]);
    }
    /*Ket thuc in dia chi*/
}
void tim_max(int arr[], int n)
{
    int max = arr[0];
    /*Bắt đầu tim max*/
    for(int i = 0; i < n; i++){
        /*Thay đổi giá trị nếu có số lớn hơn*/
        if(max < arr[i])
        {
            max = arr[i];
        }        
    }
    /*Kết thúc tìm max*/
    printf("Gia tri max la: %d \n", max);
}
void tim_min(int arr[], int n)
{
    int min = arr[0];
    /*Bắt đầu tìm min*/
    for(int i = 0; i < n; i++)
    {
       /*Đổi giá trị min*/
        if( min > arr[i])
        {
            min = arr[i];
            
        }
        
    }
    /*Kết thúc tìm min*/
    printf("Gia tri min la: %d \n", min);
}
int main()
{
    int arr[100];
    int n = -1;
    while((n < 5))
    {
        printf("nhap gia tri n: ");
        scanf("%d", &n);
    }
    nhap_array(arr, n);
    in(arr, n);
    tim_max(arr, n);
    tim_min(arr, n);
}