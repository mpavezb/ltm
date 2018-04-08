

/*

rosrun rosprolog rosprolog pkg-name
rosrun rosprolog rosprolog knowrob_common

use_module(library('module-name')).
use_module('path/to/module-name').

register_ros_package('pkg-name').

*/

% Export the perception of an object to an OWL file
export_object(knowrob:'Drawer1', 'path/to/object.owl').
export_object(knowrob:'Drawer1', 'exp_drawer1.owl').

% Export the definition of an object class to an OWL file
export_object_class(knowrob:'Drawer', 'path/to/objclass.owl').
export_object_class(knowrob:'Drawer', 'exp_drawer.owl').

% Export the map as the set of all perceptions of objects to an OWL file
export_map(ias_semantic_map:'SemanticEnvironmentMap0', 'path/to/map.owl').
export_map(ias_semantic_map:'SemanticEnvironmentMap0', 'exp_map.owl').

% Export an action specification (TBOX) to an OWL file
export_action(knowrob:'PickingUpAnObject', 'path/to/action.owl').
export_action(knowrob:'PickingUpAnObject', 'exp_action.owl').


%% OK
owl_parse('package://knowrob_maps/owl/ccrl2_semantic_map.owl').

%% OK
owl_subclass_of(A, knowrob:'FoodOrDrink').

%% OK
owl_has(A, rdf:type, knowrob:'Drawer').


%% rdf_has(S,P,O) Subject, Predicate, Object
%% owl_has(S,P,O) Subject, Predicate, Object

%% FAIL
owl_has(knowrob:'Drawer1', P, O).

%% OK
rdf_has(knowrob:'Drawer1', P, O).

%% ---------------------------------------------------------------------------------

%% OK
%% rosrun rosprolog rosprolog knowrob_basics_tutorial
owl_parse('package://knowrob_maps/owl/ccrl2_semantic_map.owl').
owl_parse('package://knowrob_basics_tutorial/owl/ccrl2_map_objects.owl').


%% Query for objects with certain properties
%% ================================================================

%% OK
owl_individual_of(Map, knowrob:'SemanticEnvironmentMap').
owl_individual_of(A, knowrob:'FoodUtensil').
owl_individual_of(A, knowrob:'FoodVessel').
owl_individual_of(A, knowrob:'FoodOrDrink').

% OK . perishable objects
owl_individual_of(A, knowrob:'Perishable').

% OK . HandTools
owl_individual_of(A, knowrob:'HandTool').

% OK . FoodVessels
owl_individual_of(A, knowrob:'FoodVessel').

% FAIL everything with a handle:
owl_has(A, knowrob:properPhysicalParts, H), owl_individual_of(H,  knowrob:'Handle').
%% expected: (and others)
%% A = map_obj:'Dishwasher37',
%% H = map_obj:'Handle145'.


%% Querying for qualitative spatial relations
%% ================================================================

%% FAIL
rdf_triple(knowrob:'in-ContGeneric', map_obj:sausage1, C).
%% expected
%% C = knowrob:'Refrigerator67'

%% FAIL
rdf_triple(knowrob:'in-ContGeneric', O, knowrob:'Refrigerator67').
%% expected:
%% O = map_obj:'cheese1' ;
%% O = map_obj:'milk1' ;
%% O = map_obj:'sausage1'

%% FAIL
rdf_triple(knowrob:'on-Physical', A, knowrob:'Dishwasher37').
%% expected:
%% A = map_obj:'cup1' ;
%% A = knowrob:'CounterTop205'



%% Inferring likely storage locations
%% ================================================================

%% Querying for likely storage locations
%% -----------------------------------

%% OK
storagePlaceFor(Place, map_obj:'butter1').

%% OK 
storagePlaceFor(knowrob:'Refrigerator67', Obj).

%% OK
storagePlaceForBecause(Place, map_obj:'butter1', Because).

%% OK
storagePlaceForBecause(knowrob:'Refrigerator67', Obj, Because).


%% Checking for objects that are not at their storage location
%% -----------------------------------

%% FAIL!
rdf_triple(knowrob:'in-ContGeneric', Obj, ActualPlace), 
    owl_individual_of(Obj, knowrob:'FoodOrDrink'), 
    storagePlaceFor(StoragePlace,Obj), 
    ActualPlace\=StoragePlace.
%% expected
%%  Obj = map_obj:'butter1',
%%  ActualPlace = knowrob:'Drawer31',
%%  StoragePlace = knowrob:'Refrigerator67' ;
%%  Obj = map_obj:'buttermilk1',
%%  ActualPlace = knowrob:'Oven19',
%%  StoragePlace = knowrob:'Refrigerator67'


%% Reading the object component hierarchy
%% ================================================================
owl_has(knowrob:'Refrigerator67', knowrob:properPhysicalParts, P).
%% expected
%% P = knowrob:'Door70' ;
%% P = knowrob:'Handle160' ;
%% P = knowrob:'Hinge70'
%% got
%% P = knowrob:'Door70' ;
%% P = knowrob:'Hinge70' ;


%% Reading articulation information
%% ================================================================

%% FAIL
create_joint_information('HingedJoint', roboearth:'IkeaExpedit2x40', roboearth:'IkeaExpeditDoor13', 
                           [1,0,0,1,0,1,0,1,0,0,1,1,0,0,0,1], [], '0.33', '0.1', '0.5', Joint).


%% FAIL
read_joint_information(Joint, Type, Parent, Child, Pose, Direction, Radius, Qmin, Qmax).

%% FAIL 
update_joint_information(Joint, 'HingedJoint', [1,0,0,2,0,1,0,2,0,0,1,2,0,0,0,1], [1,2,3], 0.32, 1.2, 1.74).


%% ==============================================================
%% Reasoning about actions
%% ==============================================================
%% not important!

%% rosrun rosprolog rosprolog knowrob_basics_tutorial
owl_parse('package://knowrob_actions/owl/pancake-making.owl').

%% Query for a sequence of actions that fulfils the ordering constraints
%% ----------------------------------------------------------------------
%% FAIL
class_properties(make_pancakes:'MakingPancakes', knowrob:subAction, Sub).

%% OK
plan_subevents(make_pancakes:'MakingPancakes', Sub).

%% OK?
plan_subevents_recursive(pancake:'MakingPancakes', Sub).


%% Query for action properties: fromLocation, toLocation, objectActedOn,...
%% ----------------------------------------------------------------------

%% FAIl
class_properties(make_pancakes:'PourDoughOntoPancakeMaker', P, O).

%% FAIL
class_properties(pancake:'MixFlourAndMilk', knowrob:objectActedOn, O).

%% Reasoning about action requirements
%% -------------------------------------------------

%% OK
register_ros_package(knowrob_srdl).

%% FAIL
srdl2:required_cap_for_action(pancake:'MakingPancakes', Cap).

%% OK
owl_parse('package://knowrob_srdl/owl/PR2.owl').

%% OK???
missing_cap_for_action(make_pancakes:'MakingPancakes', pr2:'PR2Robot1', M).



%% ======================================================================
%% Reason using computables
%% ======================================================================

%% Qualitative spatial relations
%% ----------------------------------

%% OK
rdf_assert(knowrob:cup1, knowrob:'on-Physical', knowrob:table0).


%% rosrun rosprolog rosprolog knowrob_map_data
%%  ?- owl_parse('package://knowrob_map_data/owl/ias_semantic_map.owl'),
%%     rdf_db:rdf_register_ns(ias_map, 'http://knowrob.org/kb/ias_semantic_map.owl#',  [keep(true)]).

%% OK?
rdf_triple(knowrob:'on-Physical', Top, Bottom).

%% FAIL
rdf_triple(knowrob:'on-Physical', Top, ias_map:cupboard7).



%% ======================================================================
%% Representing units of measure
%% ======================================================================

%% roscd knowrob_common
%% rosrun rosprolog rosprolog knowrob_common

%% OK
owl_parse('package://knowrob_common/owl/knowrob_units.owl').

%% FAIL
consult('prolog/knowrob_units.pl').
%% got
%% ERROR: source_sink `'prolog/knowrob_units.pl'' does not exist

%% OK
% read information that is asserted for a test instance
rdf_has('http://knowrob.org/kb/knowrob_units.owl#test-inst',
        'http://knowrob.org/kb/knowrob_units.owl#length', O).
%% O = literal(type('http://qudt.org/vocab/unit#Centimeter','12.0')) .

%% FAIL
% manual conversion into other units
convert_to_unit($O, 'http://qudt.org/vocab/unit#Kilometer', P).
%% P = 0.00012.
%% got
%% ERROR: toplevel: Undefined procedure: convert_to_unit/3 (DWIM could not correct goal)

%% FAIL
convert_to_unit($O, 'http://qudt.org/vocab/unit#Meter', P).
%% P = 0.12.

%% FAIL
convert_to_unit($O, 'http://qudt.org/vocab/unit#Millimeter', P).
%% P = 120.0.


%% FAIL
 % transparent conversion during the query  
 ?- rdf_triple('http://knowrob.org/kb/knowrob_units.owl#length', 
               'http://knowrob.org/kb/knowrob_units.owl#test-inst', 
                literal(type('http://qudt.org/vocab/unit#Meter', Val))).
 Val = 0.12 ;
 
%% FAIL 
 ?- rdf_triple('http://knowrob.org/kb/knowrob_units.owl#length', 
               'http://knowrob.org/kb/knowrob_units.owl#test-inst', 
                literal(type('http://qudt.org/vocab/unit#Kilometer', Val))).
 Val = 0.00012 ;