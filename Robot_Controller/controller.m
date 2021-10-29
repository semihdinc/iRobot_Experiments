function u = controller(qtilde,qr,ur)
% the controller
% u = [v omega]^T
% qtilde: posture error
% qr    : reference (desired) posture

u = zeros(2,1);

% for now
vr     = ur(1,1);
omegar = ur(2,1);

% According to the vision system: qtilde = q - qr, so
q = qtilde + qr;

xr     = qr(1,1);
yr     = qr(2,1);
thetar = qr(3,1);

x     = q(1,1);
y     = q(2,1);
theta = q(3,1);

s = sin(theta);
c = cos(theta);

%% -------------------------Error Hesaplaniyor----------------------------
% Burada temelde 3 tane olan hata parametreleri 4 e cikariliyor. 
% theta parametresi es ve ec olarak iki ayri hata parametresine cevriliyor

% Eq.(4), third element!
etheta = thetar-theta;

% Local errors from Eq.(10)
ex = c*(xr - x) + s*(yr - y);
ey = -s*(xr - x) + c*(yr - y);

es = sin(thetar-theta);
ecos = cos(thetar-theta);

ec = ecos -1;

%--------------------------------------------------------------------------
%% - u ya ulasabilmek icin ihtiyacimiz olanlar
% vb,omegab,vr,omegar: ilk ikisi hesaplaniyor son ikisi ur'den geliyor.
% k,kx,ks,a,n paper temel alinarak secilmis secilmis

% according to the paper
k = 1; % k > 0
kx = 1; % kx(t) > 0
ks = 1; % ks(t) > 0
a = 3; % a > 2
n = 0; % n = -2, -1, 0, 1 or 2

% Eq.(18)
vb = kx*ex;
omegab = k*vr*ey*(1+ec/a)^2+ks*es*((1+ec/a)^2)^n;

u = [vr*cos(etheta)+vb; omegar+omegab]; % Eq.(6)

%buradaki u degeri controllerdan cikan ve sisteme giris olarak verilen u
