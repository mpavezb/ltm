#ifndef LTM_PLUGINS_BASE_H
#define LTM_PLUGINS_BASE_H

namespace ltm {
    namespace plugin {

        class EmotionBase {
            public:
            virtual void initialize(double side_length) = 0;
            virtual bool get_emotion() = 0;
            virtual ~EmotionBase(){}

            protected:
            EmotionBase(){}
        };

        class LocationBase {
        public:
            virtual void initialize(double side_length) = 0;
            virtual bool get_location() = 0;
            virtual ~LocationBase(){}

        protected:
            LocationBase(){}
        };

        class StreamBase {
        public:
            virtual void initialize(double side_length) = 0;
            virtual bool get_stream() = 0;
            virtual ~StreamBase(){}

        protected:
            StreamBase(){}
        };

        class EntityBase {
        public:
            virtual void initialize(double side_length) = 0;
            virtual bool get_entity() = 0;
            virtual ~EntityBase(){}

        protected:
            EntityBase(){}
        };

    }
}

#endif //LTM_PLUGINS_BASE_H
