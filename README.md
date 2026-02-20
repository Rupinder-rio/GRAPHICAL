# GRAPHICAL
clc
clear all

% How to draw lines in matlab
% -x1 + 3x2 = 10
%  x1 + x2 = 6
%  x1 - x2 = 2

%% Phase I : Input Parameters
A =[1 1];
B = [5];

%% Phase II : Plot the lines
x1 = 0:1:max(B);

% Write given constraints
x11 = (B(1) - A(1,1).*x1) ./ A(1,2);
x21 = (B(2) - A(2,1).*x1) ./ A(2,2);
x31 = (B(3) - A(3,1).*x1) ./ A(3,2);

% Non-negative constraints
x11 = max(0, x11);
x21 = max(0, x21);
x31 = max(0, x31);

% Plot lines
plot(x1,x11,'r', x1,x21,'b', x1,x31,'g')

title('x1 vs x2')
xlabel('Value of x1')
ylabel('Value of x2')
%legend('-x1+3x2=10','x1+x2=6')
grid on

%% Phase III : Find corner points with axes
cx1 = find(x1 == 0);
c1 = find(x11 == 0);
Line1 = [x1([c1 cx1])' x11([c1 cx1])'];

c2 = find(x21 == 0);
Line2 = [x1([c2 cx1])' x21([c2 cx1])'];

c3 = find(x31 == 0);
Line3 = [x1([c3 cx1])' x31([c3 cx1])'];

corpt = unique([Line1; Line2;Line3], 'rows');

%% Phase IV : Find intersection points
pt = [];

for i = 1:size(A,1)
    A1 = A(i,:);
    B1 = B(i);

    for j = i+1:size(A,1)
        A2 = A(j,:);
        B2 = B(j);

        A4 = [A1; A2];
        B4 = [B1; B2];

        X = A4\B4;
        pt = [pt; X'];
    end
end

ptt = pt;

%% Phase V : Write all corner points
allpt = [ptt; corpt];
points = unique(allpt, 'rows');

%% Phase VI : Find feasible region
allpt = points;

x1 = allpt(:,1);
x2 = allpt(:,2);

% Constraint 1
cons1 = x1+x2-6;
h1 = find(cons1 > 0);
allpt(h1,:) = [];

x1 = allpt(:,1);
x2 = allpt(:,2);

% Constraint 2
cons2 = x1+3*x2-3;
h2 = find(cons2 < 0);
allpt(h2,:) = [];

x1 = allpt(:,1);
x2 = allpt(:,2);

% Constraint 3
cons3 = 2*x1+x2-8;
h3 = find(cons3 > 0);
allpt(h3,:) = [];

point = allpt;
P = unique(point,'rows');

%% Phase VII : Objective Function
% Max Z = x1 + 5x2

c = [5 4];
fn = zeros(size(P,1),1);

for i = 1:size(P,1)
    fn(i) = sum(P(i,:) .* c);
end

ver_fns = [P fn];

%% Phase VIII : Optimal Value
[Optval, optposition] = max(fn);

Optval = ver_fns(optposition,:);
OPTIMAL_BFS = array2table(Optval)
