import { Box, Typography } from "@mui/material";
import InputBtn from "components/InputBtn";

import { doc, onSnapshot } from "firebase/firestore";
import { useEffect, useState } from "react";
import { db } from "utils/fb";

const DataCollection = () => {
  const [perceivedSafety, setPerceivedSafety] = useState(99);

  useEffect(() => {
    return onSnapshot(doc(db, "participantData", "1"), (doc) => {
      setPerceivedSafety(doc.data().perceivedSafety);
    });
  }, []);

  const scoreToComment = {
    "-3": "Too unsafe",
    0: "Perfectly safe",
    3: "Overly safe",
  };

  return (
    <div>
      {-3 <= perceivedSafety && perceivedSafety <= 3 ? (
        <>
          <Typography variant="h5" sx={{ textAlign: "center", mt: "175px" }}>
            Your answer:
          </Typography>
          <Typography
            variant="h5"
            sx={{ textAlign: "center", fontSize: "100px" }}
          >
            {perceivedSafety}{" "}
            {scoreToComment[perceivedSafety] &&
              `(${scoreToComment[perceivedSafety]})`}
          </Typography>
        </>
      ) : (
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            flexDirection: "column",
            justifyContent: "center",
            pt: "10px",
            rowGap: "15px",
            flexWrap: "wrap",
            width: "100%",
          }}
        >
          <Typography
            variant="h4"
            sx={{
              textAlign: "center",
              fontSize: "30px",
            }}
          >
            How did you perceive the most recent trajectory?
          </Typography>

          <Box
            sx={{
              mt: "-25px",
              mb: "-20px",
            }}
          >
            <ul>
              <li>
                <Typography>
                  <b>-3 (too unsafe):</b> if you feel like the drone flew too
                  fast and uncomfortably close to you
                </Typography>
              </li>

              <li>
                <Typography>
                  <b>0 (perfectly safe):</b> if you think the drone flew at an
                  ideal distance away from you with an ideal velocity
                </Typography>
              </li>

              <li>
                <Typography>
                  <b>3 (overly safe):</b> if you feel like the drone flew too
                  slow and unnecessarily far away from you
                </Typography>
              </li>
            </ul>
          </Box>

          <InputBtn number={-3} text="too unsafe" />
          <InputBtn number={-2} />
          <InputBtn number={-1} />
          <InputBtn number={0} text="perfectly safe" />
          <InputBtn number={1} />
          <InputBtn number={2} />
          <InputBtn number={3} text="overly safe" />
        </Box>
      )}
    </div>
  );
};

export default DataCollection;
