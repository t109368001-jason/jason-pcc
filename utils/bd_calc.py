from .bd import bd_rate, bd_psnr

R1 = [2, 4, 6, 8]
PSNR1 = [1, 2, 3, 4]
R2 = [1, 2, 3, 4]
PSNR2 = [1, 2, 3, 4]

print('BD-RATE: ', bd_rate(R1, PSNR1, R2, PSNR2))
print('BD-PSNR: ', bd_psnr(R1, PSNR1, R2, PSNR2))
