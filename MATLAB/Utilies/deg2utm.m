function  [x,y,utmzone] = deg2utm(Lat,Lon)
% -------------------------------------------------------------------------
% [x,y,utmzone] = deg2utm(Lat,Lon)
%
% Description: Function to convert lat/lon vectors into UTM coordinates (WGS84).
% Some code has been extracted from deg2utm.m function by Rafael Palacios.
%
% Inputs:
%    Lat: Latitude vector.   Degrees.  +ddd.ddddd  WGS84
%    Lon: Longitude vector.  Degrees.  +ddd.ddddd  WGS84
%
% Outputs:
%    x, y , utmzone.   See example
%
% Example 1:
%    Lat=[40.3154333; 46.283900; 37.577833; 28.645650; 38.855550; 25.061783];
%    Lon=[-3.4857166; 7.8012333; -119.95525; -17.759533; -94.7990166; 121.640266];
%    [x,y,utmzone] = deg2utm(Lat,Lon);
%    fprintf('%7.0f ',x)
%       458731  407653  239027  230253  343898  362850
%    fprintf('%7.0f ',y)
%      4462881 5126290 4163083 3171843 4302285 2772478
%    utmzone =
%       30 T
%       32 T
%       11 S
%       28 R
%       15 S
%       51 R
%
% Example 2: If you have Lat/Lon coordinates in Degrees, Minutes and Seconds
%    LatDMS=[40 18 55.56; 46 17 2.04];
%    LonDMS=[-3 29  8.58;  7 48 4.44];
%    Lat=dms2deg(mat2dms(LatDMS)); %convert into degrees
%    Lon=dms2deg(mat2dms(LonDMS)); %convert into degrees
%    [x,y,utmzone] = deg2utm(Lat,Lon)
%
% Authors: 
%   Erwin Nindl, Rafael Palacious
%
% Version history by Erwin Nindl:
%   Nov/13: removed main-loop and vectorised all calculations
%
% Version history by Rafael Palacios:
%   Apr/06, Jun/06, Aug/06, Aug/06
%   Aug/06: fixed a problem (found by Rodolphe Dewarrat) related to southern 
%     hemisphere coordinates. 
%   Aug/06: corrected m-Lint warnings
%---------------------------------------------------------------------------

% Argument checking
%
error(nargchk(2, 2, nargin));  %2 arguments required
n1=length(Lat);
n2=length(Lon);
if (n1~=n2)
   error('Lat and Lon vectors should have the same length');
end


% Memory pre-allocation
%
x=zeros(n1,1);
y=zeros(n1,1);
utmzone(n1,:)='60 X';

invalid_ids = isnan(Lat) & isnan(Lon);
x(invalid_ids) = NaN;
y(invalid_ids) = NaN;


% Avoid Loop
%

% constants
sa = 6378137.000000 ; sb = 6356752.314245;
%e = ( ( ( sa ^ 2 ) - ( sb ^ 2 ) ) ^ 0.5 ) / sa;
e2 = ( ( ( sa ^ 2 ) - ( sb ^ 2 ) ) ^ 0.5 ) / sb;
e2cuadrada = e2 ^ 2;
c = ( sa ^ 2 ) / sb;
%alpha = ( sa - sb ) / sa;             %f
%ablandamiento = 1 / alpha;   % 1/f
lat = Lat .* ( pi / 180 );
lon = Lon .* ( pi / 180 );

Huso = fix( ( Lon ./ 6 ) + 31);
S = ( ( Huso .* 6 ) - 183 );
deltaS = lon - ( S .* ( pi / 180 ) );

Letra = char(zeros(n1,1));
Letra(:) = 'X';
Letra(Lat<-72) = 'C';
Letra(Lat<-64 & Lat>=-72) = 'D';
Letra(Lat<-56 & Lat>=-64) = 'E';
Letra(Lat<-48 & Lat>=-56) = 'F';
Letra(Lat<-40 & Lat>=-48) = 'G';
Letra(Lat<-32 & Lat>=-40) = 'H';
Letra(Lat<-24 & Lat>=-32) = 'J';
Letra(Lat<-16 & Lat>=-24) = 'K';
Letra(Lat<-8 & Lat>=-16) = 'L';
Letra(Lat<0 & Lat>=-8) = 'M';
Letra(Lat<8 & Lat>=0) = 'N';
Letra(Lat<16 & Lat>=8) = 'P';
Letra(Lat<24 & Lat>=16) = 'Q';
Letra(Lat<32 & Lat>=24) = 'R';
Letra(Lat<40 & Lat>=32) = 'S';
Letra(Lat<48 & Lat>=40) = 'T';
Letra(Lat<56 & Lat>=48) = 'U';
Letra(Lat<64 & Lat>=56) = 'V';
Letra(Lat<72 & Lat>=64) = 'W';

a = cos(lat) .* sin(deltaS);
epsilon = 0.5 * log( ( 1 +  a) ./ ( 1 - a ) );
nu = atan( tan(lat) ./ cos(deltaS) ) - lat;
v = ( c ./ ( ( 1 + ( e2cuadrada .* ( cos(lat) ) .^ 2 ) ) ) .^ 0.5 ) .* 0.9996;
ta = ( e2cuadrada ./ 2 ) * epsilon .^ 2 .* ( cos(lat) ) .^ 2;
a1 = sin( 2 .* lat );
a2 = a1 .* ( cos(lat) ) .^ 2;
j2 = lat + ( a1 ./ 2 );
j4 = ( ( 3 .* j2 ) + a2 ) ./ 4;
j6 = ( ( 5 .* j4 ) + ( a2 .* ( cos(lat) ) .^ 2) ) ./ 3;
alfa = ( 3 ./ 4 ) .* e2cuadrada;
beta = ( 5 ./ 3 ) .* alfa .^ 2;
gama = ( 35 ./ 27 ) .* alfa .^ 3;
Bm = 0.9996 .* c .* ( lat - alfa .* j2 + beta .* j4 - gama .* j6 );
xx = epsilon .* v .* ( 1 + ( ta ./ 3 ) ) + 500000;
yy = nu .* v .* ( 1 + ta ) + Bm;

yy(yy<0) = 9999999 + yy(yy<0);

x = xx;
y = yy;

for i=1:n1
  utmzone(i,:)=sprintf('%02d %c',Huso(i),Letra(i));
end

%eof
