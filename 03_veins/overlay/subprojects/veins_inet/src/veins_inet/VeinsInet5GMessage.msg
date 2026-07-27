import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;

class VeinsInet5GMessage extends inet::FieldsChunk
{
    string messageKind;
    long messageId;
    string senderId;
    string targetId;
    string tlsId;
    string tlsState;
    string authLabel;
    string senderRole;
    string purposeTag;
    double timeSent;
    double posX;
    double posY;
    double posZ;
    double speed;
    double velX;
    double velY;
    double velZ;
    double accel;
    double timeToChange;
    char phase;
}
