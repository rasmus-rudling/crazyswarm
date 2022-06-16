import { Box, Typography } from "@mui/material";
import InputBtn from "components/InputBtn";

import { doc, onSnapshot } from "firebase/firestore";
import { useEffect, useState } from "react";
import { db } from "utils/fb";

const ResetEvaluation = () => {
  const [perceivedSafety, setPerceivedSafety] = useState(99);

  useEffect(() => {
    return onSnapshot(doc(db, "participantData", "1"), (doc) => {
      setPerceivedSafety(doc.data().perceivedSafety);
    });
  }, []);

  return (
    <div>
      <Box
        sx={{
          display: "flex",
          alignItems: "center",
          flexDirection: "column",
          justifyContent: "center",
          pt: "10px",
          rowGap: "20px",
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
          Press the button to reset the answer
        </Typography>

        <InputBtn number={99} text="Reset" />
      </Box>
    </div>
  );
};

export default ResetEvaluation;
