/// <reference lib="webworker" />
import {
    ApplicationBehaviors,
    ApplicationNode, OptionalPromise,
    registerApplicationBehavior,
    ScriptBuilder
} from '@universal-robots/contribution-api';
import { ExternalControlApplicationNode } from './external-control-application.node';

// factory is required
export const createApplicationNode = (): OptionalPromise<ExternalControlApplicationNode> => ({
    type: 'universal-robots-external-control-external-control-application',    // type is required
    version: '1.0.0',     // version is required
    port: 50002,
    robotIP: '192.168.56.1' //the hostname of the internal daemon docker is "servicegateway"
});

// generatePreamble is optional
const generatePreambleScriptCode = (node: ExternalControlApplicationNode): OptionalPromise<ScriptBuilder> => {
    const builder = new ScriptBuilder();
    return builder;
};

// upgradeNode is optional
const upgradeApplicationNode
    = (loadedNode: ApplicationNode, defaultNode: ExternalControlApplicationNode): ExternalControlApplicationNode =>
        defaultNode;

// downgradeNode is optional
const downgradeApplicationNode
    = (loadedNode: ApplicationNode, defaultNode: ExternalControlApplicationNode): ExternalControlApplicationNode =>
        defaultNode;

const behaviors: ApplicationBehaviors = {
    factory: createApplicationNode,
    generatePreamble: generatePreambleScriptCode,
    upgradeNode: upgradeApplicationNode,
    downgradeNode: downgradeApplicationNode
};

registerApplicationBehavior(behaviors);