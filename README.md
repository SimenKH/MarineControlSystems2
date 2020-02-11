# MarineControlSystems2
Upload all work done in the lab work shall be uploaded here for easy syncronisation. Label all files appropriately


There can be some useful files at https://github.com/shivjeetrai/NTNU/tree/master/TMR4243
Though the project has been altered significantly since this was uploaded.

The report for Case A is written using overleaf (LaTeX) at https://www.overleaf.com/project/5e3e8aa997cde10001842603
You should all have gotten an invite to edit at your student e-mails, but send me an e-mail at simenkh@stud.ntnu.no if this is not the e-mail you use for overleaf.



09/02/2020, 10:45 Simen has found out what a psuedoinverse is:
Certain matricies cant be properly inverted, the psuedoinverse is the "as close to possible to what an inverse would look like". 


11/02/2020, 14:17
Simen got answers on a mail from the TA
Mail sent:
Hi,

We have started doing case study A, but we have encountered some problems:

We do not understand what is asked in Task 2.4, what is meant by "propose a procedure beta..."?
What is meant by devise a procedure in Task 3?
Can you post the slides from the information session so that we can see when you have office hours?
Thank you in advance
Simen K Helgesen (and the rest of group 3)

Answer recieved:
In the short introduction of Part I in case A, it is stated that you can only send u_cmd and alpha_cmd to the actuators that are in a certain valid range. While the approach utilizing the Moore-Penrose Inverse proposed just before task 2.4 is simple enough, it does not ensure that u_\ast is valid. Your job in task 2.4 is essentially to find a function beta which takes in the possibly invalid u_\ast and given alpha and produces as output valid u_cmd and alpha_cmd. Saturation would be a simple example, but a practical solution to this exercise is slightly more complicated. You would for instance, with alpha_1 = alpha_2 = 0 (VSP in forward direction), want to be able to drive backwards.
Task 3.1 is essentially the same as task 2.4, but this time you also need to invert the transformation that your are given. Recall that you solve for u^\check_\ast in task 3 (equation 10). Then you can utilize this to find (u_\ast,alpha_\ast), as these quantities are related to u^check by the given transformation (equation 8). Then you need to make valid (u_cmd,alpha_cmd) from (u_\ast,alpha_\ast). Ass a hint, the solution would probably involve atan2 to find the angle alpha.  
The office hours will be posted.
 

Hope this helps,

 

Henrik

