struct file *pantech_fopen(const char *filename, int flags, int mode);
void pantech_fremove(const char *filename);
void pantech_fclose(struct file *filp);
int pantech_fseek(struct file *filp, int offset, int whence);
int pantech_ftell(struct file *filp);
int pantech_fread(struct file *filp, char *buf, int len);
int pantech_fgetc(struct file *filp);
int pantech_fgets(struct file *filp, char *str, int size);

int pantech_fwrite(struct file *filp, char *buf, int len);
int pantech_fputc(struct file *filp, int ch);
int pantech_fputs(struct file *filp, char *str);
int pantech_fprintf(struct file *filp, const char *fmt, ...);

