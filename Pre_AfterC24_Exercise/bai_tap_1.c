#include <stdio.h>
void nhap_array(int *arr[], int n)
{
    /*Bắt đầu gắn giá trị cho mảng*/
    for(int i = 0;i < n; i++)
    {
        printf("arr[%d]: ", i);
        scanf("%d", &arr[i]);
    }
    /*Kết thúc gắn giá trị cho mảng*/
}
void in(int *arr[], int n)
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
int main()
{
    int *arr[100];
    int n = -1;
    printf("nhap gia tri n: ");
    scanf("%d", &n);
    /*Bắt đầu kiểm tra giá trị n theo đề bài */
    while((n <= 0) || (n > 16))
    {
        printf("nhap lai gia tri n: ");
        scanf("%d", &n);
    }
    /*Kết thúc kiểm tra*/
    nhap_array(arr, n);
    in(arr, n);
}