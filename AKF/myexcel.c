#include "xlsxwriter.h"
 
int main() {
 
    lxw_workbook  *workbook  = workbook_new("myexcel.xlsx");
    lxw_worksheet *worksheet = workbook_add_worksheet(workbook, NULL);
    int row = 0;
    int col = 0;
 
    worksheet_write_string(worksheet, row, col, "Hello me!", NULL);
 
    return workbook_close(workbook);
}