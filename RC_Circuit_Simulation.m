close all
t=0:0.1:100;
tau = 0.10;
ts = input("Enter Switch Time: ")
yo = 1-exp(-(ts/tau));
y = zeros(length(t));
for i = 1:length(t)
    if (t(i)<ts)
        y(i) = 1-exp(-t(i)/tau)
    else
        y(i) = yo*exp(-(t(i)-ts)/tau)
    end
end
plot(t,y)
grid on