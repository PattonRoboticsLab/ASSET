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


function [K, F] = assembleCableFE(mesh, E, A, rho, g, torque)
%ASSEMBLECABLEFE Assembles the stiffness matrix and force vector for a cable finite element model
%   [K, F] = ASSEMBLECABLEFE(MESH, E, A, RHO, G, TORQUE) assembles the
%   stiffness matrix and force vector for a cable finite element model with
%   the specified MESH, modulus of elasticity E, cross-sectional area A,
%   density RHO, gravitational acceleration G, and torque vector TORQUE.


function [K, F] = assembleObjectFE(mesh, E, nu, rho, g)
%ASSEMBLEOBJECTFE Assembles the stiffness matrix and force vector for an object finite element model
%   [K, F] = ASSEMBLEOBJECTFE(MESH, E, NU, RHO, G) assembles the stiffness
%   matrix and force vector for an object finite element model with the
%   specified MESH, Young's modulus E, Poisson's ratio NU, density RHO,
%   and gravitational acceleration G.


function [K, F] = assembleGlobalFE(mesh, cables, objects, E, nu, rho, g, T)
%ASSEMBLEGLOBALFE Assembles the global stiffness matrix and force vector for a cable-object system finite element model
%   [K, F] = ASSEMBLEGLOBALFE(MESH, CABLES, OBJECTS, E, NU, RHO, G, T)
%   assembles the global stiffness matrix and force vector for a
%   cable-object system finite element model with the specified MESH,
%   cable and object structures CABLES and OBJECTS, Young's modulus E,
%   Poisson's ratio NU, density RHO, gravitational acceleration G, and
%   torque vector T.

