
#define LCD_ST7920_COL (128U)
#define LCD_ST7920_ROW (64U)

unsigned char lcd_graph_buf[LCD_ST7920_COL][LCD_ST7920_ROW];

void lcd_st7920_init(void);
void lcd_st7920_configure(void);


