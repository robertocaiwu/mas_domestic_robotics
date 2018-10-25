#!/usr/bin/env python3

import io
import os
import sys
import wave
from speech_recognition import AudioData
import numpy as np
import struct

# sys.path.insert(0, '{0}/speech/py-kaldi-asr'.format(os.path.expanduser('~')))
from kaldiasr.nnet3 import KaldiNNet3OnlineModel, KaldiNNet3OnlineDecoder

class KaldiRecognizer():
    def __init__(self):
        """
        Creates a new ``Recognizer`` instance, which represents a collection of speech recognition
        functionalities.

        This class assumes you have Kaldi <https://github.com/kaldi-asr/kaldi> and a python
        wrapper for Kaldi <https://github.com/gooofy/py-kaldi-asr> already installed.
        """
        self.kaldi_model = None
        self.decoder = None

    def load_kaldi_model(self, model_directory          = None ,
                               language                 = 'de-DE',
                               model_type               = 'model',
                               acoustic_scale           = 1, # nnet3: 0.1
                               beam                     = 7.0, # nnet3: 14.0
                               frame_subsampling_factor = 3):  # nnet3: 1
        """
        The recognition language is determined by ``language``, an RFC5646 language tag like
        ``"en-US"``, defaulting to US English. Out of the box, only ``en-US`` and ``de-DE``is
        supported. For additional languages, go to Kaldi documentation <http://kaldi-asr.org/doc/>
        for training new models.
        """
        assert isinstance(model_directory, (type(""), type(u""))) or model_directory is  None, \
            'Use a valid directory path'
        assert isinstance(language, (type(""), type(u""))) and len(language) == 5, \
            'Use a RFC5646 language tag like ``"en-US"`` or ``de-DE``'
        assert isinstance(model_type, (type(""), type(u""))), 'Invalid model type'
        assert isinstance(acoustic_scale, int), 'The acoustic scale used in decoding'
        assert isinstance(beam, float), 'Lattice beam'
        assert isinstance(frame_subsampling_factor, int), 'Frame shifting factor'

        if model_directory is None:
        	if language == 'en-US':
        		model_directory = '{0}/speech/py-kaldi-asr/data/models/nnet3_en' \
    			    .format(os.path.expanduser('~'))
        	elif language == 'de-DE':
        	    model_directory = '{0}/speech/py-kaldi-asr/data/models/nnet3_de' \
        		    .format(os.path.expanduser('~'))

        # import the PocketSphinx speech recognition module
        try:
            print('loading {0}...'.format(model_type))
            self.kaldi_model = KaldiNNet3OnlineModel(model_directory,
                                         model_type,
                                         acoustic_scale = acoustic_scale,
                                         beam = beam,
                                         frame_subsampling_factor = frame_subsampling_factor)
            print('loading {0}... done.'.format(model_type))
            self.decoder = KaldiNNet3OnlineDecoder (self.kaldi_model)

        except ImportError:
            raise RequestError("missing py-kaldi-asr module: ensure that it is set up correctly.")

    def get_unpacked_data(self, audio_data=None, convert_rate=None, convert_width=None):
        """
        Returns a byte string representing the contents of a WAV file containing the audio
        represented by the ``AudioData`` instance. If ``convert_width`` is specified and the
        audio samples are not ``convert_width`` bytes each, the resulting audio is converted to
        match. If ``convert_rate`` is specified and audio sample rate is not ``convert_rate``
        Hz, the resulting audio is resampled to match. Writing these bytes directly to a file
        results in a valid `WAV file <https://en.wikipedia.org/wiki/WAV>`__.
        """
        assert isinstance(audio_data, AudioData), "``audio_data`` must be audio data"
        raw_data = audio_data.get_raw_data(convert_rate, convert_width)
        sample_rate = audio_data.sample_rate if convert_rate is None else convert_rate
        sample_width = audio_data.sample_width if convert_width is None else convert_width

        # generate the WAV file contents
        with io.BytesIO() as wav_file:
            wav_writer = wave.open(wav_file, "wb")
            # note that we can't use context manager, since that was only added in Python 3.4
            try:
                wav_writer.setframerate(sample_rate)
                wav_writer.setsampwidth(sample_width)
                wav_writer.setnchannels(1)
                wav_writer.writeframes(raw_data)
                wav_data = wav_file.getvalue()
                num_frames = wav_writer.getnframes()
            # make sure resources are cleaned up
            finally:
                wav_writer.close()
        return struct.unpack_from('<%dh' % num_frames, wav_data)

    def recognize_kaldi(self, audio_data):
        """
        Performs speech recognition on ``audio_data`` (an ``AudioData`` instance), using Kaldi.
        Returns the decoded string from Kaldi decoder.
        """
        assert isinstance(audio_data, AudioData), "``audio_data`` must be audio data"

        samples = self.get_unpacked_data(audio_data = audio_data)
        try:
            if self.decoder.decode(audio_data.sample_rate, np.array(samples, dtype=np.float32), True):
                s = self.decoder.get_decoded_string()
                return s[0]
        except:
            raise BaseException("Could not decode audio")
