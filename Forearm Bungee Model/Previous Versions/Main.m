clear all;
clc;

%% Define parameters
radius = 0.5;                   % radius of object
numElementsCable = 30;          % number of finite elements for cable
numElementsObject = 20;         % number of finite elements for object
cableDensity = 1000;            % density of cable (kg/m^3)
objectDensity = 8000;           % density of object (kg/m^3)
E = 1e9;                        % Young's modulus of cable (Pa)
load = [0; 0; -100];            % applied load on cable (N)
fixedNodes = [1:numElementsCable+1, numElementsCable+2:2*(numElementsCable+1)]; % indices of nodes that are fixed in place
center = [0; 0; 0];             % center of object

%% Generate finite element meshes
cableMesh = cylinderMesh(radius, numElementsCable, [0 0 -1]);
objectMesh = cylinderMesh(radius, numElementsObject, center);

%% Assemble stiffness matrices and load vectors
[Kcable, Fcable] = assembleCableFE(cableMesh, E, cableDensity, load, fixedNodes);
[Kobject, Fobject] = assembleObjectFE(objectMesh, objectDensity);

%% Combine stiffness matrices and load vectors
[K, F] = assembleGlobalFE(Kcable, Fcable, Kobject, Fobject, fixedNodes);

%% Solve for nodal displacements
U = K\F;

%% Extract cable displacements and calculate torque
cableNodes = cableMesh.Nodes;
cableElements = cableMesh.Elements;
cableDofs = reshape(1:numel(cableNodes)*3,[],3)';
cableDisplacements = U(cableDofs);
torque = zeros(3,1);
for i = 1:size(cableElements,1)
    elNodes = cableElements(i,:);
    elDisp = cableDisplacements(:,elNodes);
    elNodesPos = cableNodes(:,elNodes);
    elForce = elDisp(:,2:3)-elDisp(:,1:2); % force on element
    elPos = mean(elNodesPos,2)-center; % position of element relative to object center
    torque = torque + cross(elPos, elForce); % calculate torque contribution from element
end

%% Visualize torque field
[X,Y,Z] = meshgrid(-radius:0.1:radius, -radius:0.1:radius, -radius:0.1:radius);
torqueField = zeros(size(X));
for i = 1:numel(X)
    pos = [X(i); Y(i); Z(i)];
    if norm(pos-center) <= radius
        force = Fobject(cableDofs(:,1))'; % use the first fixed cable node as a proxy for the force acting on the object
        torqueField(i) = dot(cross(pos-center, force), [0;0;1]); % calculate torque acting on the object
    end
end
figure
slice(X,Y,Z,torqueField,[0],[0],[0])
xlabel('X (m)')
ylabel

function mesh = cylinderMesh(radius, numElements, center)
%CYLINDERMESH Generates a finite element mesh for a cylinder
%   MESH = CYLINDERMESH(RADIUS, NUMELEMENTS, CENTER) generates a finite
%   element mesh for a cylinder with the specified RADIUS and NUMELEMENTS.
%   CENTER is a 3-by-1 vector specifying the center of the cylinder.

% Generate nodes
theta = linspace(0, 2*pi, numElements+1)';
x = radius*cos(theta);
y = radius*sin(theta);
z = linspace(0, 1, 3)';
[X,Y,Z] = meshgrid(x,y,z);
nodes = [X(:) Y(:) Z(:)]';

% Translate nodes to center of cylinder
nodes = nodes + center(:);

% Generate elements
numNodes = numel(x)*numel(y)*numel(z);
elems = zeros(numElements*numel(z), 4);
for i = 1:numel(z)-1
    for j = 1:numElements
        n1 = (i-1)*numNodes + (j-1)*numel(y) + 1;
        n2 = (i-1)*numNodes + j*numel(y) + 1;
        n3 = i*numNodes + j*numel(y) + 1;
        n4 = i*numNodes + (j-1)*numel(y) + 1;
        elems((i-1)*numElements+j,:) = [n1 n2 n3 n4];
    end
end

% Remove duplicate nodes
[uniqueNodes, ~, elemIdx] = unique(nodes', 'rows');
elems = elemIdx(elems);

mesh.Nodes = uniqueNodes';
mesh.Elements = elems;
end

function [K, F] = assembleCableFE(mesh, E, A, rho, g, torque)
%ASSEMBLECABLEFE Assembles the stiffness matrix and force vector for a cable finite element model
%   [K, F] = ASSEMBLECABLEFE(MESH, E, A, RHO, G, TORQUE) assembles the
%   stiffness matrix and force vector for a cable finite element model with
%   the specified MESH, modulus of elasticity E, cross-sectional area A,
%   density RHO, gravitational acceleration G, and torque vector TORQUE.

% Number of nodes and elements
numNodes = size(mesh.Nodes, 2);
numElems = size(mesh.Elements, 1);

% Initialize global stiffness matrix and force vector
K = zeros(numNodes*3, numNodes*3);
F = zeros(numNodes*3, 1);

% Assemble element matrices and force vectors
for i = 1:numElems
    % Get element nodes
    nodes = mesh.Elements(i,:);
    x = mesh.Nodes(1,nodes);
    y = mesh.Nodes(2,nodes);
    z = mesh.Nodes(3,nodes);
    
    % Element length and direction vector
    L = norm([x(2)-x(1) y(2)-y(1) z(2)-z(1)]);
    l = (x(2:3)-x(1:2))'/L;
    
    % Element stiffness matrix
    k = (E*A/L)*[1 -1; -1 1];
    K11 = l*k*l';
    K22 = K11;
    K12 = -K11;
    K21 = -K12;
    Ke = [K11 K12; K21 K22];
    
    % Element force vector due to gravity
    Fe_g = rho*A*L*g/2*[0; 0; 1; 0; 0; 1];
    
    % Element force vector due to torque
    Fe_t = zeros(6,1);
    if ~isempty(torque)
        p = mean([x;y;z],2); % element centroid
        t = torque(:); % torque vector
        r = p - mean(mesh.Nodes,2); % vector from center of object to element centroid
        Fe_t = [-cross(r,l)*t; cross(r,l)*t];
    end
    
    % Assemble global stiffness matrix and force vector
    idx = repmat(nodes(:)', 6, 1);
    idx = idx(:);
    K(idx,idx) = K(idx,idx) + Ke;
    F(idx) = F(idx) + Fe_g + Fe_t;
end
end

function [K, F] = assembleObjectFE(mesh, E, nu, rho, g)
%ASSEMBLEOBJECTFE Assembles the stiffness matrix and force vector for an object finite element model
%   [K, F] = ASSEMBLEOBJECTFE(MESH, E, NU, RHO, G) assembles the stiffness
%   matrix and force vector for an object finite element model with the
%   specified MESH, Young's modulus E, Poisson's ratio NU, density RHO,
%   and gravitational acceleration G.

% Number of nodes and elements
numNodes = size(mesh.Nodes, 2);
numElems = size(mesh.Elements, 1);

% Initialize global stiffness matrix and force vector
K = zeros(numNodes*3, numNodes*3);
F = zeros(numNodes*3, 1);

% Compute elasticity tensor
C = getElasticityTensor(E, nu);

% Assemble element matrices and force vectors
for i = 1:numElems
    % Get element nodes
    nodes = mesh.Elements(i,:);
    x = mesh.Nodes(1,nodes);
    y = mesh.Nodes(2,nodes);
    z = mesh.Nodes(3,nodes);
    
    % Compute element stiffness matrix and force vector due to gravity
    [Ke, Fe_g] = getElementStiffnessAndForce(x, y, z, C, rho, g);
    
    % Assemble global stiffness matrix and force vector
    idx = repmat(nodes(:)', 9, 1);
    idx = idx(:);
    K(idx,idx) = K(idx,idx) + Ke;
    F(idx) = F(idx) + Fe_g;
end
end

function [K, F] = assembleGlobalFE(mesh, cables, objects, E, nu, rho, g, T)
%ASSEMBLEGLOBALFE Assembles the global stiffness matrix and force vector for a cable-object system finite element model
%   [K, F] = ASSEMBLEGLOBALFE(MESH, CABLES, OBJECTS, E, NU, RHO, G, T)
%   assembles the global stiffness matrix and force vector for a
%   cable-object system finite element model with the specified MESH,
%   cable and object structures CABLES and OBJECTS, Young's modulus E,
%   Poisson's ratio NU, density RHO, gravitational acceleration G, and
%   torque vector T.

% Number of nodes and cables
numNodes = size(mesh.Nodes, 2);
numCables = length(cables);

% Assemble object stiffness matrix and force vector
[K_obj, F_obj] = assembleObjectFE(mesh, E, nu, rho, g);

% Assemble cable stiffness matrix and force vector
[K_cable, F_cable] = assembleCableFE(mesh, cables);

% Initialize global stiffness matrix and force vector
K = K_cable + K_obj;
F = F_cable + F_obj;

% Add torque contribution to force vector
for i = 1:numCables
    % Get cable nodes
    nodes = cables(i).Nodes;
    
    % Compute force vector contribution due to torque
    Fe_t = getCableTorqueForce(nodes, T(:,i));
    
    % Assemble into global force vector
    idx = [3*nodes(1)-2, 3*nodes(1)-1, 3*nodes(1), ...
           3*nodes(2)-2, 3*nodes(2)-1, 3*nodes(2)]';
    F(idx) = F(idx) + Fe_t;
end
end

