import { ApplicationNode } from '@universal-robots/contribution-api';

export interface ExternalControlApplicationNode extends ApplicationNode {
  type: string;
  version: string;
  port: number;
  robotIP: string;
}
