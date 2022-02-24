
import pandas as pd
import smtplib

your_email = ""
your_password = ""

server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
server.ehlo()
server.login(your_email, your_password)

email_list = pd.read_excel('C:/Users/robot/bjorg/Mass Email List.xlsx')

names = email_list['Corporation']
emails = email_list['Email']

#for i in range(len(emails)):
#	name = names[i]
#	email = emails[i]
#
#	message = "Hello " + name
#
#	server.sendmail(your_email, [email], message)
server.close()