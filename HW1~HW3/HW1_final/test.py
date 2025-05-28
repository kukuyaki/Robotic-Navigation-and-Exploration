#pop up a window with a message
import tkinter as tk


window = tk.Tk()
test = tk.Button(text="test")
test.pack(side="top")
window.title('GUI')
window.geometry('380x400')
window.resizable(False, False)
window.iconbitmap('')
window.mainloop()

